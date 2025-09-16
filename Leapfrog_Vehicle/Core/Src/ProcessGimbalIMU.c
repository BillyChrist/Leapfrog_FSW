/* ******************************************************************************
 * @file  : ProcessGimbalIMU.c
 *
 * @brief : This module reads gimbal IMU data for direct measurement of thrust vector direction.
 *          Processes single IMU per axis (X-axis for pitch, Y-axis for roll).
 * 
 *  Created on: Sep 12, 2025
 *      Author: BillyChrist
 *      Contributors: William Henrie
 */

/* Includes ------------------------------------------------------------------*/
#include "ProcessGimbalIMU.h"
#include "heartbeat.h"
#include <stdio.h>    // Required for printf
#include <stdbool.h>  // Required for bool type
#include <math.h>

/* Constants -----------------------------------------------------------------*/
#define GIMBAL_CALIBRATION_SAMPLES 100

/* Global Variables ----------------------------------------------------------*/
// Gimbal IMU data - single IMU per axis
float gimbal_x_axis_deg = 0.0f;            // X-axis gimbal rotation angle (degrees
float gimbal_y_axis_deg = 0.0f;            // Y-axis gimbal rotation angle (degrees)
float gimbal_x_axis_velocity_degs = 0.0f;  // X-axis gimbal angular velocity (deg/s)
float gimbal_y_axis_velocity_degs = 0.0f;  // Y-axis gimbal angular velocity (deg/s)
float gimbal_x_axis_accel_g = 0.0f;        // X-axis gimbal acceleration (g)
float gimbal_y_axis_accel_g = 0.0f;        // Y-axis gimbal acceleration (g) 

// IMU readiness flag
bool gimbal_imu_ready = false;

// UART receive buffers for gimbal IMUs
uint8_t gimbal_pitch_imu_rx_buf[IMU_PACKET_LEN];  // X-axis IMU (pitch)
uint8_t gimbal_roll_imu_rx_buf[IMU_PACKET_LEN];   // Y-axis IMU (roll)

// Calibration storage for gimbal IMUs
float gimbal_x_axis_offset = 0.0f;       // X-axis gimbal IMU offset
float gimbal_y_axis_offset = 0.0f;       // Y-axis gimbal IMU offset
float gimbal_x_axis_vel_offset = 0.0f;   // X-axis velocity offset
float gimbal_y_axis_vel_offset = 0.0f;   // Y-axis velocity offset
float gimbal_x_axis_accel_offset = 0.0f; // X-axis acceleration offset
float gimbal_y_axis_accel_offset = 0.0f; // Y-axis acceleration offset

// Calibration status
TVC_CalibrationStatus gimbal_imu_calibration_status = TVC_CALIBRATING;

/* Function Implementations --------------------------------------------------*/

/**
 * @brief Reset gimbal IMU (placeholder function)
 */
void resetGimbalIMU(void) {
	// ** Placeholder Function, No reset capability for current IMU
    // TODO Reset capability will rely on routing power through the relays, and then turning them off/on for a reset... with a delay.
    
    printf(" \rResetting Gimbal IMU...\n");

    // Set relay GPIO off
    HAL_Delay(10000);
    // Set relay GPIO off

    HAL_Delay(100);

    printf("\rGimbal IMU Reset complete.\n");
}

/**
 * @brief Calibrate gimbal IMU sensors and actuator positions (simplified for single IMU per axis)
 * @return TVC_CalibrationStatus: TVC_CALIBRATING or TVC_CALIBRATION_COMPLETE
 */
// TODO: Add actuator position calibration
TVC_CalibrationStatus calibrateGimbalIMU(void) {
    static int sampleCount = 0;
    static float pitch_angle_sum = 0.0f;
    static float roll_angle_sum = 0.0f;
    static float pitch_vel_sum = 0.0f;
    static float roll_vel_sum = 0.0f;
    static float pitch_accel_sum = 0.0f;
    static float roll_accel_sum = 0.0f;

    if (sampleCount == 0) {
        printf("\rGimbal IMU Calibration started...\n\r");
    }

    // Process both gimbal IMU data
    processGimbalIMUData(gimbal_pitch_imu_rx_buf, GIMBAL_PITCH_IMU);
    processGimbalIMUData(gimbal_roll_imu_rx_buf, GIMBAL_ROLL_IMU);

    // Sum up the data for averaging (using global variables directly)
    pitch_angle_sum += gimbal_x_axis_deg;
    roll_angle_sum += gimbal_y_axis_deg;
    pitch_vel_sum += gimbal_x_axis_velocity_degs;
    roll_vel_sum += gimbal_y_axis_velocity_degs;
    pitch_accel_sum += gimbal_x_axis_accel_g;
    roll_accel_sum += gimbal_y_axis_accel_g;

    sampleCount++;

    if (sampleCount >= GIMBAL_CALIBRATION_SAMPLES) {
        // Calculate offsets
        gimbal_x_axis_offset = pitch_angle_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_y_axis_offset = roll_angle_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_x_axis_vel_offset = pitch_vel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_y_axis_vel_offset = roll_vel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_x_axis_accel_offset = pitch_accel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_y_axis_accel_offset = roll_accel_sum / GIMBAL_CALIBRATION_SAMPLES;

        printf("\r\n----- Gimbal IMU Offsets -----\r\n");
        printf("X-Axis Angle: %.4f   Y-Axis Angle: %.4f\r\n", gimbal_x_axis_offset, gimbal_y_axis_offset);
        printf("X-Axis Vel:   %.4f   Y-Axis Vel:   %.4f\r\n", gimbal_x_axis_vel_offset, gimbal_y_axis_vel_offset);
        printf("X-Axis Accel: %.4f   Y-Axis Accel: %.4f\r\n", gimbal_x_axis_accel_offset, gimbal_y_axis_accel_offset);

        printf("\r\n[TVC] Gimbal IMU Calibration Complete. System entering normal operation...\r\n");

        // Reset for future use
        sampleCount = 0;
        pitch_angle_sum = 0.0f;
        roll_angle_sum = 0.0f;
        pitch_vel_sum = 0.0f;
        roll_vel_sum = 0.0f;
        pitch_accel_sum = 0.0f;
        roll_accel_sum = 0.0f;

        gimbal_imu_ready = true;
        gimbal_imu_calibration_status = TVC_CALIBRATION_COMPLETE;
        return TVC_CALIBRATION_COMPLETE;
    }

    return TVC_CALIBRATING;
}

/**
 * @brief Process gimbal IMU data packet (simplified for single IMU per axis)
 * @param buffer: Pointer to UART receive buffer
 * @param imu_type: GIMBAL_PITCH_IMU or GIMBAL_ROLL_IMU
 */
void processGimbalIMUData(uint8_t *buffer, int imu_type) {
    // Calculate checksum
    uint8_t checksum = 0x55 + buffer[1];
    for (int i = 2; i <= 9; i++) {
        checksum += buffer[i];
    }

    if (checksum != buffer[10]) {
        return;  // Discard packet on checksum fail
    }

    // Process data based on IMU type and packet type
    switch (buffer[1]) {
        case 0x51:  // Acceleration Packet
            if (imu_type == GIMBAL_PITCH_IMU) {
                // X-axis IMU - store Y-axis acceleration for X-axis gimbal
                gimbal_x_axis_accel_g = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 16;
                // Apply acceleration corrections
                gimbal_x_axis_accel_g = (gimbal_x_axis_accel_g > 10) ? (gimbal_x_axis_accel_g - 20) : gimbal_x_axis_accel_g;
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU - store X-axis acceleration for Y-axis gimbal
                gimbal_y_axis_accel_g = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 16;
                // Apply acceleration corrections
                gimbal_y_axis_accel_g = (gimbal_y_axis_accel_g > 10) ? (gimbal_y_axis_accel_g - 20) : gimbal_y_axis_accel_g;
            }
            break;

        case 0x52:  // Angular Velocity Packet
            if (imu_type == GIMBAL_PITCH_IMU) {
                // X-axis IMU - store Y-axis angular velocity for X-axis gimbal
                gimbal_x_axis_velocity_degs = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 2000;
                // Apply angular velocity corrections
                gimbal_x_axis_velocity_degs = (gimbal_x_axis_velocity_degs > 1000) ? (gimbal_x_axis_velocity_degs - 4000) : gimbal_x_axis_velocity_degs;
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU - store X-axis angular velocity for Y-axis gimbal
                gimbal_y_axis_velocity_degs = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 2000;
                // Apply angular velocity corrections
                gimbal_y_axis_velocity_degs = (gimbal_y_axis_velocity_degs > 1000) ? (gimbal_y_axis_velocity_degs - 4000) : gimbal_y_axis_velocity_degs;
            }
            break;

        case 0x53:  // Angle Packet
            if (imu_type == GIMBAL_PITCH_IMU) {
                // X-axis IMU - store pitch angle for X-axis gimbal
                gimbal_x_axis_deg = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 180;
                // Apply angle wrapping
                gimbal_x_axis_deg = wrapAngle(gimbal_x_axis_deg);
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU - store pitch angle for Y-axis gimbal
                gimbal_y_axis_deg = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 180;
                // Apply angle wrapping
                gimbal_y_axis_deg = wrapAngle(gimbal_y_axis_deg);
            }
            break;
    }
}

/**
 * @brief Update gimbal IMU data (simplified for single IMU per axis)
 * Applies calibration offsets to the global gimbal data
 */
void updateGimbalIMUData(void) {
    // Apply calibration offsets to the global gimbal data
    gimbal_x_axis_deg -= gimbal_x_axis_offset;
    gimbal_y_axis_deg -= gimbal_y_axis_offset;
    
    // Apply velocity offsets
    gimbal_x_axis_velocity_degs -= gimbal_x_axis_vel_offset;
    gimbal_y_axis_velocity_degs -= gimbal_y_axis_vel_offset;
    
    // Apply acceleration offsets
    gimbal_x_axis_accel_g -= gimbal_x_axis_accel_offset;
    gimbal_y_axis_accel_g -= gimbal_y_axis_accel_offset;
}


/**
 * @brief Check if gimbal IMU data is valid
 * @return true if data is valid and recent, false otherwise
 */
bool isGimbalIMUDataValid(void) {
    return gimbal_imu_ready && (gimbal_imu_calibration_status == TVC_CALIBRATION_COMPLETE);
}