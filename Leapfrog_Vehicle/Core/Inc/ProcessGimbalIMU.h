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
    CALIBRATING = 0,
    CALIBRATION_COMPLETE = 1
} CalibrationStatus;

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
} IMUPacket;

/* Global Variables ----------------------------------------------------------*/
// Gimbal IMU data - single IMU per axis
extern float gimbal_pitch_deg;        // Gimbal pitch angle from X-axis IMU
extern float gimbal_roll_deg;         // Gimbal roll angle from Y-axis IMU
extern float gimbal_pitch_velocity_degs;  // Gimbal pitch angular velocity
extern float gimbal_roll_velocity_degs;   // Gimbal roll angular velocity
extern float gimbal_pitch_accel_g;    // Gimbal pitch acceleration
extern float gimbal_roll_accel_g;     // Gimbal roll acceleration

// IMU readiness flag
extern bool gimbal_imu_ready;

// Calibration status
extern CalibrationStatus gimbal_imu_calibration_status;

/* Function Declarations -----------------------------------------------------*/
// Calibration functions
CalibrationStatus calibrateGimbalIMU_RTOS(void);
void resetGimbalIMU(void);

// Data processing functions
void processGimbalIMUData(uint8_t *buffer, int imu_type);
void updateGimbalIMUData(void);

// Data access functions
void getGimbalAngles(float *pitch_deg, float *roll_deg);
void getGimbalVelocities(float *pitch_velocity_degs, float *roll_velocity_degs);
void getGimbalAccelerations(float *pitch_accel_g, float *roll_accel_g);
bool isGimbalIMUDataValid(void);

// Utility functions
float wrapAngle(float angle);

#endif /* PROCESSGIMBALIMU_H */