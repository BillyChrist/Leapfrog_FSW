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

// Gimbal IMU data packets
IMUPacket gimbal_pitch_packet;  // X-axis IMU packet
IMUPacket gimbal_roll_packet;   // Y-axis IMU packet

// Calibration storage for gimbal IMUs
float gimbal_pitch_offset = 0.0f;     // Pitch IMU offset
float gimbal_roll_offset = 0.0f;      // Roll IMU offset
float gimbal_pitch_vel_offset = 0.0f; // Pitch velocity offset
float gimbal_roll_vel_offset = 0.0f;  // Roll velocity offset
float gimbal_pitch_accel_offset = 0.0f; // Pitch acceleration offset
float gimbal_roll_accel_offset = 0.0f;  // Roll acceleration offset

// Calibration status
CalibrationStatus gimbal_imu_calibration_status = CALIBRATING;

/* Function Implementations --------------------------------------------------*/

/**
 * @brief Angle wrapping logic
 * @param angle: Input angle in degrees
 * @return Wrapped angle in range [-180, 180]
 */
float wrapAngle(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0)
        angle += 360.0f;
    return angle - 180.0f;
}

/**
 * @brief Reset gimbal IMU (placeholder function)
 */
void resetGimbalIMU(void) {
	// ** Placeholder Function, No reset capability for current IMU
    printf(" \rResetting Gimbal IMU...\n");

    // Set relay GPIO off
    HAL_Delay(10000);
    // Set relay GPIO off

    HAL_Delay(100);

    printf("\rGimbal IMU Reset complete.\n");
}

/**
 * @brief Calibrate gimbal IMU sensors (simplified for single IMU per axis)
 * @return CalibrationStatus: CALIBRATING or CALIBRATION_COMPLETE
 */
CalibrationStatus calibrateGimbalIMU_RTOS(void) {
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

    // Sum up the data for averaging
    pitch_angle_sum += gimbal_pitch_packet.pitch_deg;
    roll_angle_sum += gimbal_roll_packet.roll_deg;
    pitch_vel_sum += gimbal_pitch_packet.angvel_y_degs;  // Y-axis angular velocity for pitch
    roll_vel_sum += gimbal_roll_packet.angvel_x_degs;    // X-axis angular velocity for roll
    pitch_accel_sum += gimbal_pitch_packet.acc_y_g;      // Y-axis acceleration for pitch
    roll_accel_sum += gimbal_roll_packet.acc_x_g;        // X-axis acceleration for roll

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
        gimbal_imu_calibration_status = CALIBRATION_COMPLETE;
        return CALIBRATION_COMPLETE;
    }

    return CALIBRATING;
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

    // Select appropriate packet based on IMU type
    IMUPacket *pkt = (imu_type == GIMBAL_PITCH_IMU) ? &gimbal_pitch_packet : &gimbal_roll_packet;

    switch (buffer[1]) {
        case 0x51:  // Acceleration Packet
            pkt->acc_x_g   = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 16;
            pkt->acc_y_g   = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 16;
            pkt->acc_z_g   = ((buffer[7] << 8) | buffer[6]) / 32768.0f * 16;
            pkt->temp_degc = ((buffer[9] << 8) | buffer[8]) / 340.0f + 36.25f;
            break;

        case 0x52:  // Angular Velocity Packet
            pkt->angvel_x_degs = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 2000;
            pkt->angvel_y_degs = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 2000;
            pkt->angvel_z_degs = ((buffer[7] << 8) | buffer[6]) / 32768.0f * 2000;
            pkt->temp_degc     = ((buffer[9] << 8) | buffer[8]) / 340.0f + 36.25f;
            break;

        case 0x53:  // Angle Packet
            pkt->roll_deg  = ((buffer[3] << 8) | buffer[2]) / 32768.0f * 180;
            pkt->pitch_deg = ((buffer[5] << 8) | buffer[4]) / 32768.0f * 180;
            pkt->yaw_deg   = ((buffer[7] << 8) | buffer[6]) / 32768.0f * 180;
            pkt->temp_degc = ((buffer[9] << 8) | buffer[8]) / 340.0f + 36.25f;
            break;
    }

    // Apply angle wrapping
    pkt->roll_deg = wrapAngle(pkt->roll_deg);
    pkt->pitch_deg = wrapAngle(pkt->pitch_deg);
    pkt->yaw_deg = wrapAngle(pkt->yaw_deg);

    // Apply angular velocity corrections
    pkt->angvel_x_degs = (pkt->angvel_x_degs > 1000) ? (pkt->angvel_x_degs - 4000) : pkt->angvel_x_degs;
    pkt->angvel_y_degs = (pkt->angvel_y_degs > 1000) ? (pkt->angvel_y_degs - 4000) : pkt->angvel_y_degs;
    pkt->angvel_z_degs = (pkt->angvel_z_degs > 1000) ? (pkt->angvel_z_degs - 4000) : pkt->angvel_z_degs;

    // Apply acceleration corrections
    pkt->acc_x_g = (pkt->acc_x_g > 10) ? (pkt->acc_x_g - 20) : pkt->acc_x_g;
    pkt->acc_y_g = (pkt->acc_y_g > 10) ? (pkt->acc_y_g - 20) : pkt->acc_y_g;
    pkt->acc_z_g = (pkt->acc_z_g > 10) ? (pkt->acc_z_g - 20) : pkt->acc_z_g;
}

/**
 * @brief Update gimbal IMU data (simplified for single IMU per axis)
 * Applies calibration offsets and updates global gimbal data
 */
void updateGimbalIMUData(void) {
    // Apply calibration offsets and update gimbal data
    gimbal_pitch_deg = gimbal_pitch_packet.pitch_deg - gimbal_pitch_offset;
    gimbal_roll_deg = gimbal_roll_packet.roll_deg - gimbal_roll_offset;
    
    // Update angular velocities (apply offsets)
    gimbal_pitch_velocity_degs = gimbal_pitch_packet.angvel_y_degs - gimbal_pitch_vel_offset;
    gimbal_roll_velocity_degs = gimbal_roll_packet.angvel_x_degs - gimbal_roll_vel_offset;
    
    // Update accelerations (apply offsets)
    gimbal_pitch_accel_g = gimbal_pitch_packet.acc_y_g - gimbal_pitch_accel_offset;
    gimbal_roll_accel_g = gimbal_roll_packet.acc_x_g - gimbal_roll_accel_offset;
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
    return gimbal_imu_ready && (gimbal_imu_calibration_status == CALIBRATION_COMPLETE);
}