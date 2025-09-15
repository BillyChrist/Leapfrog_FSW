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
float gimbal_pitch_deg = 0.0f;        // Gimbal pitch angle from X-axis IMU
float gimbal_roll_deg = 0.0f;         // Gimbal roll angle from Y-axis IMU
float gimbal_pitch_velocity_degs = 0.0f;  // Gimbal pitch angular velocity
float gimbal_roll_velocity_degs = 0.0f;   // Gimbal roll angular velocity
float gimbal_pitch_accel_g = 0.0f;    // Gimbal pitch acceleration
float gimbal_roll_accel_g = 0.0f;     // Gimbal roll acceleration

// IMU readiness flag
bool gimbal_imu_ready = false;

// UART receive buffers for gimbal IMUs
uint8_t gimbal_pitch_imu_rx_buf[IMU_PACKET_LEN];  // X-axis IMU (pitch)
uint8_t gimbal_roll_imu_rx_buf[IMU_PACKET_LEN];   // Y-axis IMU (roll)

// Calibration storage for gimbal IMUs
float gimbal_pitch_offset = 0.0f;     // Pitch IMU offset
float gimbal_roll_offset = 0.0f;      // Roll IMU offset
float gimbal_pitch_vel_offset = 0.0f; // Pitch velocity offset
float gimbal_roll_vel_offset = 0.0f;  // Roll velocity offset
float gimbal_pitch_accel_offset = 0.0f; // Pitch acceleration offset
float gimbal_roll_accel_offset = 0.0f;  // Roll acceleration offset

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
    pitch_angle_sum += gimbal_pitch_deg;
    roll_angle_sum += gimbal_roll_deg;
    pitch_vel_sum += gimbal_pitch_velocity_degs;
    roll_vel_sum += gimbal_roll_velocity_degs;
    pitch_accel_sum += gimbal_pitch_accel_g;
    roll_accel_sum += gimbal_roll_accel_g;

    sampleCount++;

    if (sampleCount >= GIMBAL_CALIBRATION_SAMPLES) {
        // Calculate offsets
        gimbal_pitch_offset = pitch_angle_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_roll_offset = roll_angle_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_pitch_vel_offset = pitch_vel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_roll_vel_offset = roll_vel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_pitch_accel_offset = pitch_accel_sum / GIMBAL_CALIBRATION_SAMPLES;
        gimbal_roll_accel_offset = roll_accel_sum / GIMBAL_CALIBRATION_SAMPLES;

        printf("\r\n----- Gimbal IMU Offsets -----\r\n");
        printf("Pitch Angle: %.4f   Roll Angle: %.4f\r\n", gimbal_pitch_offset, gimbal_roll_offset);
        printf("Pitch Vel:   %.4f   Roll Vel:   %.4f\r\n", gimbal_pitch_vel_offset, gimbal_roll_vel_offset);
        printf("Pitch Accel: %.4f   Roll Accel: %.4f\r\n", gimbal_pitch_accel_offset, gimbal_roll_accel_offset);

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
                // X-axis IMU for pitch - store Y-axis acceleration for pitch
                gimbal_pitch_accel_g = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 16;
                // Apply acceleration corrections
                gimbal_pitch_accel_g = (gimbal_pitch_accel_g > 10) ? (gimbal_pitch_accel_g - 20) : gimbal_pitch_accel_g;
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU for roll - store X-axis acceleration for roll
                gimbal_roll_accel_g = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 16;
                // Apply acceleration corrections
                gimbal_roll_accel_g = (gimbal_roll_accel_g > 10) ? (gimbal_roll_accel_g - 20) : gimbal_roll_accel_g;
            }
            break;

        case 0x52:  // Angular Velocity Packet
            if (imu_type == GIMBAL_PITCH_IMU) {
                // X-axis IMU for pitch - store Y-axis angular velocity for pitch
                gimbal_pitch_velocity_degs = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 2000;
                // Apply angular velocity corrections
                gimbal_pitch_velocity_degs = (gimbal_pitch_velocity_degs > 1000) ? (gimbal_pitch_velocity_degs - 4000) : gimbal_pitch_velocity_degs;
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU for roll - store X-axis angular velocity for roll
                gimbal_roll_velocity_degs = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 2000;
                // Apply angular velocity corrections
                gimbal_roll_velocity_degs = (gimbal_roll_velocity_degs > 1000) ? (gimbal_roll_velocity_degs - 4000) : gimbal_roll_velocity_degs;
            }
            break;

        case 0x53:  // Angle Packet
            if (imu_type == GIMBAL_PITCH_IMU) {
                // X-axis IMU for pitch - store pitch angle
                gimbal_pitch_deg = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 180;
                // Apply angle wrapping
                gimbal_pitch_deg = wrapAngle(gimbal_pitch_deg);
            } else if (imu_type == GIMBAL_ROLL_IMU) {
                // Y-axis IMU for roll - store roll angle
                gimbal_roll_deg = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 180;
                // Apply angle wrapping
                gimbal_roll_deg = wrapAngle(gimbal_roll_deg);
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
    gimbal_pitch_deg -= gimbal_pitch_offset;
    gimbal_roll_deg -= gimbal_roll_offset;
    
    // Apply velocity offsets
    gimbal_pitch_velocity_degs -= gimbal_pitch_vel_offset;
    gimbal_roll_velocity_degs -= gimbal_roll_vel_offset;
    
    // Apply acceleration offsets
    gimbal_pitch_accel_g -= gimbal_pitch_accel_offset;
    gimbal_roll_accel_g -= gimbal_roll_accel_offset;
}

/**
 * @brief Get current gimbal angles
 * @param pitch_deg: Current gimbal pitch angle (output)
 * @param roll_deg: Current gimbal roll angle (output)
 */
void getGimbalAngles(float *pitch_deg, float *roll_deg) {
    *pitch_deg = gimbal_pitch_deg;
    *roll_deg = gimbal_roll_deg;
}

/**
 * @brief Get current gimbal angular velocities
 * @param pitch_velocity_degs: Current gimbal pitch angular velocity (output)
 * @param roll_velocity_degs: Current gimbal roll angular velocity (output)
 */
void getGimbalVelocities(float *pitch_velocity_degs, float *roll_velocity_degs) {
    *pitch_velocity_degs = gimbal_pitch_velocity_degs;
    *roll_velocity_degs = gimbal_roll_velocity_degs;
}

/**
 * @brief Get current gimbal accelerations
 * @param pitch_accel_g: Current gimbal pitch acceleration (output)
 * @param roll_accel_g: Current gimbal roll acceleration (output)
 */
void getGimbalAccelerations(float *pitch_accel_g, float *roll_accel_g) {
    *pitch_accel_g = gimbal_pitch_accel_g;
    *roll_accel_g = gimbal_roll_accel_g;
}

/**
 * @brief Check if gimbal IMU data is valid
 * @return true if data is valid and recent, false otherwise
 */
bool isGimbalIMUDataValid(void) {
    return gimbal_imu_ready && (gimbal_imu_calibration_status == TVC_CALIBRATION_COMPLETE);
}