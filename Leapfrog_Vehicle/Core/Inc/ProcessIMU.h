/*
 * processIMU.h
 *
 *  Created on: Oct 8, 2024
 *      Author: BillyChrist
 */

#ifndef PROCESSIMU_H
#define PROCESSIMU_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>  // Required for bool type


// Define packet length for IMU data
#define IMU_PACKET_LEN 11

// Scalable IMU redundancy, update these numbers to reflect the number of IMU sensors used.
#define NUM_IMUS 3
#define NUM_AXES 3

// IMU-related global variables (updated by processIMU.c, used in ACS_PID.c and main.c)
extern UART_HandleTypeDef huart3;  // IMU1
extern UART_HandleTypeDef huart4;  // IMU2
extern UART_HandleTypeDef huart6;  // IMU3

//extern float processed_roll_deg;
//extern float processed_pitch_deg;
//extern float processed_yaw_deg;

extern float current_roll_deg;
extern float current_pitch_deg;
extern float current_yaw_deg;

extern float current_angvel_x_degs;
extern float current_angvel_y_degs;
extern float current_angvel_z_degs;

extern float current_acc_x_g;
extern float current_acc_y_g;
extern float current_acc_z_g;

// IMU Data Packet Structure
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float angvel_x_degs;
    float angvel_y_degs;
    float angvel_z_degs;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float temp_degc;
} IMUPacketBase;

// Aliases for clarity — compiler treats all as IMUPacketBase
typedef IMUPacketBase IMUPacket;
typedef IMUPacketBase IMU1Packet;
typedef IMUPacketBase IMU2Packet;
typedef IMUPacketBase IMU3Packet;

// IMU buffers & packets
extern uint8_t imu1RxBuf[IMU_PACKET_LEN];
extern uint8_t imu2RxBuf[IMU_PACKET_LEN];
extern uint8_t imu3RxBuf[IMU_PACKET_LEN];

extern IMUPacket packet;
extern IMU1Packet packet1;
extern IMU2Packet packet2;
extern IMU3Packet packet3;

extern float IMU1Data[3][3];
extern float IMU2Data[3][3];
extern float IMU3Data[3][3];


// Processed (Cross-Checked) Values
extern float processed_roll_deg;
extern float processed_pitch_deg;
extern float processed_yaw_deg;
extern float processed_angvel_x_degs;
extern float processed_angvel_y_degs;
extern float processed_angvel_z_degs;
extern float processed_acc_x_g;
extern float processed_acc_y_g;
extern float processed_acc_z_g;

// TVC-specific IMU data (dedicated copies to avoid race conditions)
extern float tvc_roll_deg;
extern float tvc_pitch_deg;
extern float tvc_yaw_deg;
extern float tvc_angvel_x_degs;
extern float tvc_angvel_y_degs;
extern float tvc_angvel_z_degs;
extern float tvc_acc_x_g;
extern float tvc_acc_y_g;
extern float tvc_acc_z_g;

// TVC-specific velocity data (calculated from IMU acceleration)
extern float tvc_velocity_north_ms;
extern float tvc_velocity_east_ms;
extern float tvc_velocity_up_ms;

// Raw Roll, Pitch, Yaw from each IMU
extern float imu_roll[NUM_IMUS];
extern float imu_pitch[NUM_IMUS];
extern float imu_yaw[NUM_IMUS];


// Function prototypes for IMU operations
typedef enum {
    CALIBRATING,
    CALIBRATION_COMPLETE
} CalibrationStatus;


void resetIMU(void); // Resets the IMU to a known state
void calibrateIMU_Init(void);  // Startup version
CalibrationStatus calibrateIMU_RTOS(void);

extern CalibrationStatus imuCalibrationStatus;
extern bool imu_ready;


// Processes a single IMU data packet and updates the global IMU data variables
void processIMUData(uint8_t *buffer, int IMUid);
void IMUCrossCheck(void);

// Utility functions
float wrapAngle(float angle);


#endif /* PROCESSIMU_H */
