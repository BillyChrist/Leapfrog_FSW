/* ******************************************************************************
 * @file  : ProcessGimbalIMU.h
 *
 * @brief : Header file for gimbal IMU processing module.
 *          Handles single IMU per axis (X-axis for pitch, Y-axis for roll).
 * 
 *  Created on: Oct 08, 2024
 *      Author: BillyChrist
 *      Contributors: William Henrie
 */

#ifndef PROCESSGIMBALIMU_H
#define PROCESSGIMBALIMU_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Constants -----------------------------------------------------------------*/
#define IMU_PACKET_LEN 11
#define GIMBAL_PITCH_IMU 1  // X-axis IMU for pitch measurement
#define GIMBAL_ROLL_IMU 2   // Y-axis IMU for roll measurement
 
/* Enums ---------------------------------------------------------------------*/
typedef enum {
    TVC_CALIBRATING = 0,
    TVC_CALIBRATION_COMPLETE = 1
} TVC_CalibrationStatus;

/* Structs -------------------------------------------------------------------*/
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
} TVC_IMUPacket;

/* Global Variables ----------------------------------------------------------*/
// Gimbal IMU data
extern float gimbal_x_axis_deg;
extern float gimbal_y_axis_deg;
extern float gimbal_x_axis_velocity_degs;
extern float gimbal_y_axis_velocity_degs;
extern float gimbal_x_axis_accel_g;
extern float gimbal_y_axis_accel_g;

// IMU readiness flag
extern bool gimbal_imu_ready;

// Calibration status
extern TVC_CalibrationStatus gimbal_imu_calibration_status;

/* Function Declarations -----------------------------------------------------*/
// Calibration functions
TVC_CalibrationStatus calibrateGimbalIMU(void);
void resetGimbalIMU(void);

// Data processing functions
void processGimbalIMUData(uint8_t *buffer, int imu_type);
void updateGimbalIMUData(void);

bool isGimbalIMUDataValid(void);

// Utility functions
float wrapAngle(float angle);

#endif /* PROCESSGIMBALIMU_H */
