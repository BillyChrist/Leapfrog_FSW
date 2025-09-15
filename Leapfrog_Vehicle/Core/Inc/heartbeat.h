/*
 * heartbeat.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Viserion
 */

#ifndef INC_HEARTBEAT_H_
#define INC_HEARTBEAT_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "main.h"
#include "altimeter.h" 		     // For altitude
#include "alt_imu_coupling.h"    // For corrected altitude data
#include "processIMU.h"		     // For IMU
#include "engine.h"              // For engine telemetry
#include "engine_PID.h"




// consider attribute packing to ensure propper padding: typedef struct __attribute__((packed))
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float angvel_x_degs;
    float angvel_y_degs;
    float angvel_z_degs;
    float tvc_a_pos;
    float tvc_b_pos;
    int32_t engine_turbine_rpm;
    int32_t engine_rpm_setpoint;
    int32_t engine_egt_c;
    float engine_pump_voltage;
    uint8_t engine_turbine_state;
    uint8_t engine_off_condition;
    uint8_t engine_throttle_percent;
    float engine_current_a;
    int16_t altitude;
    uint8_t heartbeat_counter;
    uint8_t imu_calibration_status;
    char status_message[64];     // Status message for ground station
    
    // GPS data
    float gps_latitude;          // GPS latitude in degrees
    float gps_longitude;         // GPS longitude in degrees
    float gps_altitude;          // GPS altitude in meters
    float gps_speed_ms;          // GPS ground speed in m/s
    float gps_heading_deg;       // GPS heading in degrees
    uint8_t gps_fix_status;      // GPS fix status (0=no fix, 1=GPS fix, 2=DGPS fix)
    bool gps_data_valid;         // GPS data validity flag
    float gps_position_error_north_m;  // North position error in meters
    float gps_position_error_east_m;   // East position error in meters
    float gps_velocity_north_ms;       // North velocity in m/s
    float gps_velocity_east_ms;        // East velocity in m/s
    
	// altimeter calibration
    // TVC calibration
    // GPS calibration
} STM32Response;



typedef struct {
  uint8_t engine_state;
  uint8_t tvc_state;
  uint8_t acs_state;
  float32_t tvcVal1_angular;
  float32_t tvcVal2_angular;
  float32_t engine_hover_m;
  uint8_t engine_thrust_p;
  int8_t calibrateIMU;
  uint8_t engine_safe;
  uint8_t engine_power;
  char command_string[64];    // Navigation command string from ground station
  uint8_t checksum;
} STM32Message;
/* NOTE:
 * The STM32Message structure isn't exactly what the STM32 receives from the Pi:
 * tvc_angle1, tvc_angle2, and engine_hover_m (all float32_t) are sent from the Pi as tvc_angle1_mdeg, tvc_angle2_mdeg,
 * and engine_hover_mm (all int32_t), but are processed before being put in the STM32Message structure
*/

// Command parser structure for ground station communication
typedef struct {
  char raw_command[64];      // Raw command string from ground station (e.g., "Move Forward 5m 1ms")
  char direction[16];        // Parsed direction: "forward", "backward", "left", "right"
  float distance_m;          // Parsed distance in meters
  float velocity_ms;         // Parsed velocity in m/s
  float gimbal_angle_deg;    // Calculated gimbal angle for this command
  float duration_s;          // Calculated duration based on distance/velocity
  float jetThrottle_offset;  // Calculated throttle offset for this command
  bool new_command;          // Flag for new command received
  bool executing;            // Flag for command currently executing
  uint32_t start_time_ms;    // Start time of command execution
} CommandParser;

// Status message structure for ground station communication
typedef struct {
  char status_message[64];   // Status message (e.g., "Navigation Complete", "Command Executing")
  bool status_updated;       // Flag for new status message
  uint32_t status_time_ms;   // Timestamp of status message
} StatusMessage;


//#define STM32_RESPONSE_SIZE 89 // Bytes = (4*9)+(4*2)+(4*5)+(4*4)+(1*3)+(4*1)+(2*1) = 89
// consider static_assert(sizeof(STM32Response) == 89, "STM32Response Size Mismatch"

//#define STM32_MESSAGE_SIZE 23 // Bytes = (1*3)+(4*3)+(1*8) = 23
#define STM32_MESSAGE_SIZE sizeof(STM32Message)
#define STM32_RESPONSE_SIZE sizeof(STM32Response)
#define STM32_SYNC_BYTE_COUNT 5
#define STM32_SYNC_BYTE 0x77

// Global instance of STM32Response for system-wide access
extern STM32Response stm32Response;

// Declare functions
void heartbeatDebugOutput(UART_HandleTypeDef *huart, STM32Response *resp_pkt);
//void updateIMUResponse(STM32Response *response);
void buildHeartbeatPacket(STM32Response *response);
void sendHeartbeatPacket(UART_HandleTypeDef *huart, STM32Response *response);

// Command parsing and status functions
void ProcessIncomingCommand(const char* command_string);
void ParseTVCCommand(const char* cmd);
void ParseACSCommand(const char* cmd);
void ParseEngineCommand(const char* cmd);
void ParseNavigationCommand(const char* cmd);
void SetStatusMessage(const char* message);
void ClearStatusMessage(void);
bool HasNewCommand(void);
bool HasStatusUpdate(void);

// Pi UART communication functions
void InitPiUARTReception(UART_HandleTypeDef *huart);
void ProcessPiByte(uint8_t received_byte);



#endif /* INC_HEARTBEAT_H_ */
