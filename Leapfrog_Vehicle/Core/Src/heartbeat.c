/* ******************************************************************************
 * @file  : heartbeat.c
 *
 * @brief : This module handles telemetry packaging to send up to the flight controller (raspberry Pi), and reads incoming packets from CLI.
 * 
 *  Created on: Apr 10, 2025
 *      Author: BillyChrist
 *  ******************************************************************************
 */

#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "heartbeat.h"
#include "altimeter.h"
#include "alt_imu_coupling.h"
#include "processIMU.h"
#include "engine.h"
#include "tvc.h"
#include "gps.h"

// Define variables
uint8_t timeout_counter;
STM32Response resp_pkt;
STM32Message heartbeat;

// Command parser and status message instances
CommandParser command_parser;
StatusMessage status_message;

// UART2 reception variables for Pi communication
static uint8_t pi_rx_buffer[84]; // 84 bytes: 5 sync + 79 message bytes
static uint8_t sync_byte_count = 0;
static uint8_t message_bytes_received = 0;
static bool receiving_message = false;
static uint8_t current_byte_index = 0;


void emplace_buffer(uint8_t* buf, size_t* offset, void* data, size_t sz) {
	memcpy(buf + (*offset), data, sz);
	*offset += sz;
}



// Output function to read heartbeat from terminal
void heartbeatDebugOutput(UART_HandleTypeDef *huart, STM32Response *resp_pkt){
    char uart_buffer[512]; // Buffer for UART text output

    // Clear the buffer
    memset(uart_buffer, 0, sizeof(uart_buffer));

    // Populate the buffer with telemetry data
    int len = snprintf(uart_buffer, sizeof(uart_buffer),
            "\rRPY: %.2f, %.2f, %.2f deg\r\n"
            "\rAccel XYZ: %.2f, %.2f, %.2f g\r\n"
            "\rAngVel XYZ: %.2f, %.2f, %.2f deg/s\r\n"
            "\rTVC: A=%.2f deg, B=%.2f deg\r\n"
            "\rEngine RPM: %ld / Setpoint: %ld / EGT: %ld C\r\n"
            "\rPump V: %.2f V / Throttle: %d %% / Current: %.2f A\r\n"
            "\rAltitude: %d cm\r\n"
            "\rHeartbeat Timeout: %d\r\n"
			"\rIMU Calibration Status: %s\r\n"
    		"\r\n",
            resp_pkt->roll_deg,
			resp_pkt->pitch_deg,
			resp_pkt->yaw_deg,
            resp_pkt->acc_x_g, resp_pkt->acc_y_g, resp_pkt->acc_z_g,
            resp_pkt->angvel_x_degs, resp_pkt->angvel_y_degs, resp_pkt->angvel_z_degs,
            resp_pkt->tvc_a_pos, resp_pkt->tvc_b_pos,
            (long int)resp_pkt->engine_turbine_rpm,
            (long int)resp_pkt->engine_rpm_setpoint,
            (long int)resp_pkt->engine_egt_c,
            resp_pkt->engine_pump_voltage, resp_pkt->engine_throttle_percent, resp_pkt->engine_current_a,
			resp_pkt->altitude,
            resp_pkt->heartbeat_counter,
			(resp_pkt->imu_calibration_status == CALIBRATION_COMPLETE) ? "COMPLETE" : "CALIBRATING"
			// TVC calibration
			// GPS calibration
    );


    // Transmit over UART
    HAL_UART_Transmit(huart, (uint8_t *)uart_buffer, len, HAL_MAX_DELAY);

}




void buildHeartbeatPacket(STM32Response *resp_pkt){

	// Load IMU data
	resp_pkt->roll_deg  = processed_roll_deg;
	resp_pkt->pitch_deg = processed_pitch_deg;
	resp_pkt->yaw_deg   = processed_yaw_deg;

	resp_pkt->acc_x_g = processed_acc_x_g;
	resp_pkt->acc_y_g = processed_acc_y_g;
	resp_pkt->acc_z_g = processed_acc_z_g;

	resp_pkt->angvel_x_degs = processed_angvel_x_degs;
	resp_pkt->angvel_y_degs = processed_angvel_y_degs;
	resp_pkt->angvel_z_degs = processed_angvel_z_degs;

	// Load TVC Positions
	resp_pkt->tvc_a_pos = (float)tvcVal1_angular;
	resp_pkt->tvc_b_pos = (float)tvcVal2_angular;

	// TODO Add GPS Packet!

	// Load Engine Telemetry
	resp_pkt->engine_turbine_rpm     = turbineRPM;
	resp_pkt->engine_rpm_setpoint    = rpm_setpoint;
	resp_pkt->engine_egt_c           = egt_c;
	resp_pkt->engine_pump_voltage    = pump_voltage;
	resp_pkt->engine_turbine_state   = turbine_state;
	resp_pkt->engine_off_condition   = off_condition;
	resp_pkt->engine_throttle_percent= throttle_percent;
	resp_pkt->engine_current_a       = engine_current_a;

	// Load Altimeter telemetry
	resp_pkt->altitude = (int16_t)adjusted_altitude_cm;

	// Load Timeout Counter
	resp_pkt->heartbeat_counter = timeout_counter;

	// Load calibration status (0 = CALIBRATING, 1 = CALIBRATION_COMPLETE)
	resp_pkt->imu_calibration_status = (imu_ready) ? CALIBRATION_COMPLETE : CALIBRATING;
	
	// Load status message
	if (status_message.status_updated) {
		strncpy(resp_pkt->status_message, status_message.status_message, sizeof(resp_pkt->status_message) - 1);
		resp_pkt->status_message[sizeof(resp_pkt->status_message) - 1] = '\0';
		status_message.status_updated = false;  // Clear flag after sending
	} else {
		strcpy(resp_pkt->status_message, "System Ready");
	}
	
	// Load GPS data
	extern GPS_Data latestGPSdata;
	resp_pkt->gps_latitude = latestGPSdata.latitude;
	resp_pkt->gps_longitude = latestGPSdata.longitude;
	resp_pkt->gps_altitude = latestGPSdata.altitude;
	resp_pkt->gps_speed_ms = latestGPSdata.speed_ms;
	resp_pkt->gps_heading_deg = latestGPSdata.heading_deg;
	resp_pkt->gps_fix_status = latestGPSdata.fix_status;
	resp_pkt->gps_data_valid = gpsIsDataValid(&latestGPSdata);
	resp_pkt->gps_position_error_north_m = latestGPSdata.position_error_north_m;
	resp_pkt->gps_position_error_east_m = latestGPSdata.position_error_east_m;
	resp_pkt->gps_velocity_north_ms = latestGPSdata.velocity_north_ms;
	resp_pkt->gps_velocity_east_ms = latestGPSdata.velocity_east_ms;
	
	// Cross-check GPS altitude with altimeter
	float altimeter_altitude_m = adjusted_altitude_cm / 100.0f; // Convert cm to meters
	gpsCrossCheckAltitude(&latestGPSdata, altimeter_altitude_m);
	
	// ALT calibration
	// TVC calibration
	// GPS calibration
}


// Sends the prepared heartbeat packet over UART using STM32 defined protocol
void sendHeartbeatPacket(UART_HandleTypeDef *huart, STM32Response *resp_pkt){

	uint8_t tx_buf[STM32_RESPONSE_SIZE + STM32_SYNC_BYTE_COUNT];
	memset(tx_buf, STM32_SYNC_BYTE, STM32_SYNC_BYTE_COUNT); // Sync bytes
	memset(tx_buf + STM32_SYNC_BYTE_COUNT, 0, STM32_RESPONSE_SIZE); // Clear rest

	size_t offset = STM32_SYNC_BYTE_COUNT;

	// Copy all data fields sequentially into the tx_buf
	emplace_buffer(tx_buf, &offset, &resp_pkt->roll_deg, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->pitch_deg, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->yaw_deg, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->acc_x_g, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->acc_y_g, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->acc_z_g, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->angvel_x_degs, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->angvel_y_degs, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->angvel_z_degs, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->tvc_a_pos, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->tvc_b_pos, sizeof(float));
	// TODO Add GPS packet
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_turbine_rpm, sizeof(int32_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_rpm_setpoint, sizeof(int32_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_egt_c, sizeof(int32_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_pump_voltage, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_turbine_state, sizeof(uint8_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_off_condition, sizeof(uint8_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_throttle_percent, sizeof(uint8_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->engine_current_a, sizeof(float));
	emplace_buffer(tx_buf, &offset, &resp_pkt->altitude, sizeof(int16_t));
	emplace_buffer(tx_buf, &offset, &resp_pkt->heartbeat_counter, sizeof(uint8_t));

	// Load calibration status
	emplace_buffer(tx_buf, &offset, &resp_pkt->imu_calibration_status, sizeof(uint8_t));
	// TODO implement altimeter, tvc & gps calibration
	//emplace_buffer(tx_buf, &offset, &resp_pkt->alt_calibration_status, sizeof(uint8_t));
	//emplace_buffer(tx_buf, &offset, &resp_pkt->tvc_calibration_status, sizeof(uint8_t));
	//emplace_buffer(tx_buf, &offset, &resp_pkt->gps_calibration_status, sizeof(uint8_t));



	// Transmit over UART
	HAL_UART_Transmit(huart, tx_buf, STM32_RESPONSE_SIZE + STM32_SYNC_BYTE_COUNT, HAL_MAX_DELAY);
}


// This part goes in main.c ...
//buildHeartbeatPacket(&resp_pkt, timeout_counter);
//
//#ifndef DEBUG_PRINTF
//	sendHeartbeatPacket(&huart2, &resp_pkt);
//#endif
//

/**
 * @brief Process incoming command from ground station
 * @param command_string: Raw command string (e.g., "Move Forward 5m 1ms")
 * 
 * This function parses commands and sets up the command parser for execution.
 */
void ProcessIncomingCommand(const char* command_string) {
    // Clear previous command data
    memset(&command_parser, 0, sizeof(CommandParser));
    
    // Copy raw command
    strncpy(command_parser.raw_command, command_string, sizeof(command_parser.raw_command) - 1);
    command_parser.raw_command[sizeof(command_parser.raw_command) - 1] = '\0';
    
    // Parse command type
    if (strncmp(command_string, "navigation ", 11) == 0) {
        // Navigation commands: "navigation hover", "navigation move Forward 5m 1ms", "navigation rotate 30"
        ParseNavigationCommand(command_string + 11);
    } else if (strncmp(command_string, "tvc ", 4) == 0) {
        // Direct TVC commands: "tvc enable", "tvc disable"
        ParseTVCCommand(command_string + 4);
    } else if (strncmp(command_string, "acs ", 4) == 0) {
        // Direct ACS commands: "acs enable", "acs disable"
        ParseACSCommand(command_string + 4);
    } else if (strncmp(command_string, "engine ", 7) == 0) {
        // Direct engine commands: "engine enable", "engine disable"
        ParseEngineCommand(command_string + 7);
    } else if (strncmp(command_string, "Move ", 5) == 0) {
        // Direct navigation command: "Move Forward 5m 1ms"
        ParseNavigationCommand(command_string);
    } else {
        // Unknown command format
        SetStatusMessage("Invalid Command Format");
    }
}


/**
 * @brief Parse direct TVC commands
 * @param cmd: Command string after "tvc " prefix
 */
void ParseTVCCommand(const char* cmd) {
    if (strcmp(cmd, "enable") == 0) {
        extern TVC_State tvcState;
        tvcState = SystemTVC_Automatic;
        SetStatusMessage("TVC (Auto Mode) Enabled");
    } else if (strcmp(cmd, "disable") == 0) {
        extern TVC_State tvcState;
        tvcState = SystemTVC_Disable;
        SetStatusMessage("TVC Disabled");
    } else if (strcmp(cmd, "manual") == 0) {
        extern TVC_State tvcState;
        tvcState = SystemTVC_Manual;
        SetStatusMessage("TVC (Manual Mode) Enabled");
    } else if (strcmp(cmd, "automatic") == 0) {
        extern TVC_State tvcState;
        tvcState = SystemTVC_Automatic;
        SetStatusMessage("TVC (Auto Mode) Enabled");
    } else {
        SetStatusMessage("Invalid TVC Command");
    }
}

/**
 * @brief Parse direct ACS commands
 * @param cmd: Command string after "acs " prefix
 */
void ParseACSCommand(const char* cmd) {
    if (strcmp(cmd, "enable") == 0) {
        extern ACS_State acsState;
        acsState = SystemACS_Enable;
        SetStatusMessage("ACS Enabled");
    } else if (strcmp(cmd, "disable") == 0) {
        extern ACS_State acsState;
        acsState = SystemACS_Disable;
        SetStatusMessage("ACS Disabled");
    } else {
        SetStatusMessage("Invalid ACS Command");
    }
}

/**
 * @brief Parse direct engine commands
 * @param cmd: Command string after "engine " prefix
 */
void ParseEngineCommand(const char* cmd) {
    if (strcmp(cmd, "enable") == 0) {
        extern Jet_State engineState;
        engineState = SystemJet_Automatic;
        SetStatusMessage("Engine (Auto Mode)Enabled");
    } else if (strcmp(cmd, "disable") == 0) {
        extern Jet_State engineState;
        engineState = SystemJet_Disable;
        SetStatusMessage("Engine Disabled");
    } else {
        SetStatusMessage("Invalid Engine Command");
    }
}

/**
 * @brief Parse navigation commands
 * @param cmd: Full navigation command string
 */
void ParseNavigationCommand(const char* cmd) {
    // Expected formats:
    // 1. "hover", "move [Direction] [Distance]m [Velocity]ms", "rotate [degrees]" (from "navigation" prefix)
    // 2. "Move [Direction] [Distance]m [Velocity]ms" (direct format)
    // Examples: "hover", "move Forward 5m 1ms", "rotate 30", "Move Forward 5m 1ms"
    
    char* token = strtok((char*)cmd, " ");
    if (token && strcmp(token, "hover") == 0) {
        // Enter hover mode
        extern void Hover(void);
        Hover();
        SetStatusMessage("Navigation Hover Mode");
        command_parser.new_command = true;
        command_parser.executing = true;
        command_parser.start_time_ms = HAL_GetTick();
    } else if (token && (strcmp(token, "move") == 0 || strcmp(token, "Move") == 0)) {
        // Get direction
        token = strtok(NULL, " ");
        if (token) {
            strncpy(command_parser.direction, token, sizeof(command_parser.direction) - 1);
            command_parser.direction[sizeof(command_parser.direction) - 1] = '\0';
        }
        
        // Get distance
        token = strtok(NULL, " ");
        if (token) {
            command_parser.distance_m = atof(token);
        }
        
        // Get velocity
        token = strtok(NULL, " ");
        if (token) {
            command_parser.velocity_ms = atof(token);
        }
        
        // Mark as new command
        command_parser.new_command = true;
        
        // Set status message
        SetStatusMessage("Command Received");
    } else {
        SetStatusMessage("Invalid Navigation Command");
    }
}

/**
 * @brief Set status message for ground station
 * @param message: Status message string
 */
void SetStatusMessage(const char* message) {
    strncpy(status_message.status_message, message, sizeof(status_message.status_message) - 1);
    status_message.status_message[sizeof(status_message.status_message) - 1] = '\0';
    status_message.status_updated = true;
    status_message.status_time_ms = HAL_GetTick();
}

/**
 * @brief Clear status message
 */
void ClearStatusMessage(void) {
    status_message.status_updated = false;
    memset(status_message.status_message, 0, sizeof(status_message.status_message));
}

/**
 * @brief Check if there's a new command to process
 * @return true if new command available
 */
bool HasNewCommand(void) {
    return command_parser.new_command;
}

/**
 * @brief Check if there's a status update to send
 * @return true if status update available
 */
bool HasStatusUpdate(void) {
    return status_message.status_updated;
}

/**
 * @brief Initialize UART2 reception for Pi communication
 * @param huart: UART handle for UART2
 */
void InitPiUARTReception(UART_HandleTypeDef *huart) {
    // Start UART reception in interrupt mode
    if (HAL_UART_Receive_IT(huart, pi_rx_buffer, 1) != HAL_OK) {
        printf("Pi UART2 Reception Initialization Failed!\r\n");
    } else {
        printf("Pi UART2 Reception Initialized!\r\n");
    }
}

/**
 * @brief Process received byte from Pi via UART2
 * @param received_byte: Single byte received from Pi
 */
void ProcessPiByte(uint8_t received_byte) {
    if (!receiving_message) {
        // Looking for sync bytes
        if (received_byte == STM32_SYNC_BYTE) {
            sync_byte_count++;
            if (sync_byte_count >= STM32_SYNC_BYTE_COUNT) {
                receiving_message = true;
                message_bytes_received = 0;
                current_byte_index = 0;
                sync_byte_count = 0;
            }
        } else {
            sync_byte_count = 0; // Reset if not sync byte
        }
    } else {
        // Receiving message data
        pi_rx_buffer[current_byte_index] = received_byte;
        current_byte_index++;
        message_bytes_received++;
        
        if (message_bytes_received >= 79) { // 84 total - 5 sync = 79 message bytes
            // Complete message received, unpack it
            STM32Message received_message;
            size_t offset = 0;
            
            // Unpack message (skip sync bytes, start from message data)
            received_message.engine_state = pi_rx_buffer[offset++];
            received_message.tvc_state = pi_rx_buffer[offset++];
            received_message.acs_state = pi_rx_buffer[offset++];
            
            // Unpack TVC angles (sent as int32_t, convert to float)
            int32_t tvc_angle1_mdeg = *((int32_t*)(pi_rx_buffer + offset));
            received_message.tvcVal1_angular = tvc_angle1_mdeg / 1000.0f;
            offset += 4;
            
            int32_t tvc_angle2_mdeg = *((int32_t*)(pi_rx_buffer + offset));
            received_message.tvcVal2_angular = tvc_angle2_mdeg / 1000.0f;
            offset += 4;
            
            // Unpack engine hover (sent as int32_t, convert to float)
            int32_t engine_hover_mm = *((int32_t*)(pi_rx_buffer + offset));
            received_message.engine_hover_m = engine_hover_mm / 1000.0f;
            offset += 4;
            
            received_message.engine_thrust_p = pi_rx_buffer[offset++];
            received_message.calibrateIMU = pi_rx_buffer[offset++];
            received_message.engine_safe = pi_rx_buffer[offset++];
            received_message.engine_power = pi_rx_buffer[offset++];
            
            // Unpack command string (64 bytes)
            strncpy(received_message.command_string, (char*)(pi_rx_buffer + offset), 63);
            received_message.command_string[63] = '\0';
            offset += 64;
            
            // Verify checksum
            uint8_t calculated_checksum = 0;
            for (int i = 0; i < 78; i++) { // 79 bytes - 1 checksum byte
                calculated_checksum += pi_rx_buffer[i];
            }
            uint8_t received_checksum = pi_rx_buffer[78];
            
            if (calculated_checksum == received_checksum) {
                // Valid message, put it in the queue
                extern QueueHandle_t piMessageQueueHandle;
                xQueueSend(piMessageQueueHandle, &received_message, 0);
            }
            
            // Reset for next message
            receiving_message = false;
            message_bytes_received = 0;
            current_byte_index = 0;
            sync_byte_count = 0;
        }
    }
}




