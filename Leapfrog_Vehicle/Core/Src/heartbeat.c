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
#include "main.h"
#include "heartbeat.h"
#include "altimeter.h"
#include "alt_imu_coupling.h"
#include "processIMU.h"
#include "engine.h"
#include "tvc.h"

// Define variables
uint8_t timeout_counter;
STM32Response resp_pkt;
STM32Message heartbeat;


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




