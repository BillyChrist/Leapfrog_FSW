/* ******************************************************************************
 * @file  : processIMU.c
 *
 * @brief : This module reads all IMU data, performs sensor fusion, cross check, 
 *          and implements fallback logic upon sensor failure.
 * 
 *  Created on: Oct 08, 2024
 *      Author: BillyChrist
 *      Contributors: William Henrie
 */

/* Includes ------------------------------------------------------------------*/
#include "processIMU.h"
#include "heartbeat.h"
#include <stdio.h>    // Required for printf
#include <stdbool.h>  // Required for bool type
#include <math.h>

// Variables for IMU data processing
float current_roll_deg;
float current_pitch_deg;
float current_yaw_deg;
float current_angvel_x_degs;
float current_angvel_y_degs;
float current_angvel_z_degs;
float current_acc_x_g;
float current_acc_y_g;
float current_acc_z_g;


// IMU readiness flag
bool imu_ready = false;

uint8_t imu1RxBuf[IMU_PACKET_LEN];
uint8_t imu2RxBuf[IMU_PACKET_LEN];
uint8_t imu3RxBuf[IMU_PACKET_LEN];
static int threshold = 15;

// IMU data packet
IMUPacket packet;
IMU1Packet packet1;
IMU2Packet packet2;
IMU3Packet packet3;
float IMU1Data[3][3];
float IMU2Data[3][3];
float IMU3Data[3][3];

// Declare variables for the cross-checked IMU values
float processed_roll_deg;
float processed_pitch_deg;
float processed_yaw_deg;
float processed_angvel_x_degs;
float processed_angvel_y_degs;
float processed_angvel_z_degs;
float processed_acc_x_g;
float processed_acc_y_g;
float processed_acc_z_g;

float imu_roll[NUM_IMUS];
float imu_pitch[NUM_IMUS];
float imu_yaw[NUM_IMUS];

// Calibration storage
IMUPacket imuPackets[NUM_IMUS];
float imuData[NUM_IMUS][NUM_AXES][NUM_AXES];
float imuOffsets[NUM_IMUS][NUM_AXES] = {0};  // Roll, Pitch, Yaw offsets per IMU

// Struct for response data
STM32Response imuResponse;
CalibrationStatus imuCalibrationStatus = CALIBRATING;


// Angle wrapping logic
float wrapAngle(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0)
        angle += 360.0f;
    return angle - 180.0f;
}


// Resets the IMU to a known state
void resetIMU(void) {
	// ** Placeholder Function, No reset capability for current IMU
    printf(" \rResetting IMU...\n");

    // Set relay GPIO off
    HAL_Delay(10000);
    // Set relay GPIO off

    HAL_Delay(100);

    printf("\rIMU Reset complete.\n");
}


// TODO Ignore 0 values! (first return from IMU will be 0... also sometimes they never respond...)
CalibrationStatus calibrateIMU_RTOS(void) {
    static int sampleCount = 0;
    static float imuSums[NUM_IMUS][9] = {0};  // Roll, Pitch, Yaw, AngX, AngY, AngZ, AccX, AccY, AccZ
    const int calibration_samples = 100;

    if (sampleCount == 0) {
        printf("\rIMU Calibration started...\n\r");
    }

    for (int imu = 0; imu < NUM_IMUS; imu++) {
        switch (imu) {
            case 0: processIMUData(imu1RxBuf, 1); break;
            case 1: processIMUData(imu2RxBuf, 2); break;
            case 2: processIMUData(imu3RxBuf, 3); break;
        }

        // Use the appropriate IMU data array based on which IMU we're processing
        float (*imuData)[3] = (imu == 0) ? IMU1Data : (imu == 1) ? IMU2Data : IMU3Data;

        // Sum up the wrapped angle data
        imuSums[imu][0] += imuData[0][0];  // Roll
        imuSums[imu][1] += imuData[0][1];  // Pitch
        imuSums[imu][2] += imuData[0][2];  // Yaw
        imuSums[imu][3] += imuData[1][0];  // AngVel X
        imuSums[imu][4] += imuData[1][1];  // AngVel Y
        imuSums[imu][5] += imuData[1][2];  // AngVel Z
        imuSums[imu][6] += imuData[2][0];  // Acc X
        imuSums[imu][7] += imuData[2][1];  // Acc Y
        imuSums[imu][8] += imuData[2][2];  // Acc Z
    }

    sampleCount++;

    if (sampleCount >= calibration_samples) {
        for (int imu = 0; imu < NUM_IMUS; imu++) {
            for (int var = 0; var < 9; var++) {
                imuOffsets[imu][var] = imuSums[imu][var] / calibration_samples;
            }
        }

        for (int imu = 0; imu < NUM_IMUS; imu++) {
            printf("\r\n----- IMU%d Offsets -----\r\n", imu + 1);
            printf("Roll:     %.4f   Pitch: %.4f   Yaw:    %.4f\r\n",
                imuOffsets[imu][0], imuOffsets[imu][1], imuOffsets[imu][2]);
            printf("AngVel_X: %.4f   AngVel_Y: %.4f   AngVel_Z: %.4f\r\n",
                imuOffsets[imu][3], imuOffsets[imu][4], imuOffsets[imu][5]);
            printf("Acc_X:    %.4f   Acc_Y:    %.4f   Acc_Z:    %.4f\r\n",
                imuOffsets[imu][6], imuOffsets[imu][7], imuOffsets[imu][8]);
        }

        printf("\r\n[ACS] IMU Calibration Complete. System entering normal operation...\r\n");


        sampleCount = 0; // Reset for future use
        imu_ready = true;

        // after calibration done:
        imuCalibrationStatus = CALIBRATION_COMPLETE;
        return CALIBRATION_COMPLETE;
    }

    return CALIBRATING;
}

// Processes a single IMU data packet, takes in pointer to IMU specific buffer and ID to know which IMU data was received
void processIMUData(uint8_t *buffer, int IMUid) {

    // Calculate checksum
    uint8_t checksum = 0x55 + buffer[1];
    for (int i = 2; i <= 9; i++) {
        checksum += buffer[i];
    }

    if (checksum != buffer[10]) {
        return;  // Discard packet on checksum fail
    }

    // Pointers to relevant IMU packet + data based on IMUid
    IMUPacketBase *packets[] = { &packet1, &packet2, &packet3 };
    float (*data[])[3] = { IMU1Data, IMU2Data, IMU3Data };


    if (IMUid < 1 || IMUid > 3) return;  // Safety check
    IMUPacketBase *pkt = packets[IMUid-1];
    float (*imuData)[3] = data[IMUid-1];  // Point to IMU set

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

    // Update IMUData arrays for cross-check
    imuData[0][0] = wrapAngle(pkt->roll_deg);
    imuData[0][1] = wrapAngle(pkt->pitch_deg);
    imuData[0][2] = wrapAngle(pkt->yaw_deg);

    imuData[1][0] = (pkt->angvel_x_degs > 1000) ? (pkt->angvel_x_degs - 4000) : pkt->angvel_x_degs;
    imuData[1][1] = (pkt->angvel_y_degs > 1000) ? (pkt->angvel_y_degs - 4000) : pkt->angvel_y_degs;
    imuData[1][2] = (pkt->angvel_z_degs > 1000) ? (pkt->angvel_z_degs - 4000) : pkt->angvel_z_degs;

    imuData[2][0] = (pkt->acc_x_g > 10) ? (pkt->acc_x_g - 20) : pkt->acc_x_g;
    imuData[2][1] = (pkt->acc_y_g > 10) ? (pkt->acc_y_g - 20) : pkt->acc_y_g;
    imuData[2][2] = (pkt->acc_z_g > 10) ? (pkt->acc_z_g - 20) : pkt->acc_z_g;
}

/* Compares pitch, roll, and yaw values across all three IMUs -- and -- ALSO APPLYS IMU OFFSETS
 * assigns the actual value as the average of the three IMU readings if all three IMUs agree within a range
 * discards the third IMU reading if two of the IMUs agree, but one of the IMUs deviates from the acceptable range
 * TODO if all three IMUs disagree, what do we want it to do?
 */
void IMUCrossCheck(void) {
    // Output pointers // TODO track where "current" data is coming from. Name changed? apply correct IMU output term.
    float *ang_outputs[3]    = { &current_roll_deg, 	 &current_pitch_deg, 	 &current_yaw_deg };
    float *angvel_outputs[3] = { &current_angvel_x_degs, &current_angvel_y_degs, &current_angvel_z_degs };
    float *accel_outputs[3]  = { &current_acc_x_g, 		 &current_acc_y_g, 		 &current_acc_z_g };

    // Loop through roll/pitch/yaw or X/Y/Z
    for (int i = 0; i < 3; i++) {
        /* -------------------- Roll / Pitch / Yaw -------------------- */
          float imu1_angle = IMU1Data[0][i] - imuOffsets[0][i];  // Apply offset for RPY
    	  float imu2_angle = IMU2Data[0][i] - imuOffsets[1][i];
    	  float imu3_angle = IMU3Data[0][i] - imuOffsets[2][i];

        if (fabs(imu1_angle - imu2_angle) > threshold && fabs(imu1_angle - imu3_angle) > threshold && fabs(imu2_angle - imu3_angle) < threshold) {
            *(ang_outputs[i]) = (imu2_angle + imu3_angle) / 2.0f;
        } else if (fabs(imu2_angle - imu3_angle) > threshold && fabs(imu2_angle - imu1_angle) > threshold && fabs(imu1_angle - imu3_angle) < threshold) {
            *(ang_outputs[i]) = (imu1_angle + imu3_angle) / 2.0f;
        } else if (fabs(imu3_angle - imu1_angle) > threshold && fabs(imu3_angle - imu2_angle) > threshold && fabs(imu1_angle - imu2_angle) < threshold) {
            *(ang_outputs[i]) = (imu1_angle + imu2_angle) / 2.0f;
        } else {
            *(ang_outputs[i]) = (imu1_angle + imu2_angle + imu3_angle) / 3.0f;
        }

        /* -------------------- Angular Velocity -------------------- */
        float imu1_angvel = IMU1Data[1][i] - imuOffsets[0][i+3];  // Apply offset for angular velocity
        float imu2_angvel = IMU2Data[1][i] - imuOffsets[1][i+3];
        float imu3_angvel = IMU3Data[1][i] - imuOffsets[2][i+3];

        if (fabs(imu1_angvel - imu2_angvel) > threshold && fabs(imu1_angvel - imu3_angvel) > threshold && fabs(imu2_angvel - imu3_angvel) < threshold) {
            *(angvel_outputs[i]) = (imu2_angvel + imu3_angvel) / 2.0f;
        } else if (fabs(imu2_angvel - imu3_angvel) > threshold && fabs(imu2_angvel - imu1_angvel) > threshold && fabs(imu1_angvel - imu3_angvel) < threshold) {
            *(angvel_outputs[i]) = (imu1_angvel + imu3_angvel) / 2.0f;
        } else if (fabs(imu3_angvel - imu1_angvel) > threshold && fabs(imu3_angvel - imu2_angvel) > threshold && fabs(imu1_angvel - imu2_angvel) < threshold) {
            *(angvel_outputs[i]) = (imu1_angvel + imu2_angvel) / 2.0f;
        } else {
            *(angvel_outputs[i]) = (imu1_angvel + imu2_angvel + imu3_angvel) / 3.0f;
        }

        /* -------------------- Acceleration -------------------- */
        float imu1_accel = IMU1Data[2][i]- imuOffsets[0][i+6];  // Apply offset for acceleration
        float imu2_accel = IMU2Data[2][i]- imuOffsets[1][i+6];
        float imu3_accel = IMU3Data[2][i]- imuOffsets[2][i+6];

        if (fabs(imu1_accel - imu2_accel) > threshold && fabs(imu1_accel - imu3_accel) > threshold && fabs(imu2_accel - imu3_accel) < threshold) {
            *(accel_outputs[i]) = (imu2_accel + imu3_accel) / 2.0f;
        } else if (fabs(imu2_accel - imu3_accel) > threshold && fabs(imu2_accel - imu1_accel) > threshold && fabs(imu1_accel - imu3_accel) < threshold) {
            *(accel_outputs[i]) = (imu1_accel + imu3_accel) / 2.0f;
        } else if (fabs(imu3_accel - imu1_accel) > threshold && fabs(imu3_accel - imu2_accel) > threshold && fabs(imu1_accel - imu2_accel) < threshold) {
            *(accel_outputs[i]) = (imu1_accel + imu2_accel) / 2.0f;
        } else {
            *(accel_outputs[i]) = (imu1_accel + imu2_accel + imu3_accel) / 3.0f;
        }
    }

    /* -------------------- Store raw Roll/Pitch/Yaw from IMUs for debug -------------------- */
    imu_roll[0]  = IMU1Data[0][0];
    imu_pitch[0] = IMU1Data[0][1];
    imu_yaw[0]   = IMU1Data[0][2];

    imu_roll[1]  = IMU2Data[0][0];
    imu_pitch[1] = IMU2Data[0][1];
    imu_yaw[1]   = IMU2Data[0][2];

    imu_roll[2]  = IMU3Data[0][0];
    imu_pitch[2] = IMU3Data[0][1];
    imu_yaw[2]   = IMU3Data[0][2];

    /* -------------------- Store processed Roll/Pitch/Yaw ---------------------- */
    processed_roll_deg  = *ang_outputs[0];
    processed_pitch_deg = *ang_outputs[1];
    processed_yaw_deg   = *ang_outputs[2];

    /* -------------------- Store processed Angular Velocity -------------------- */
    processed_angvel_x_degs = *angvel_outputs[0];
    processed_angvel_y_degs = *angvel_outputs[1];
    processed_angvel_z_degs = *angvel_outputs[2];

    /* -------------------- Store processed Acceleration ------------------------ */
    processed_acc_x_g = *accel_outputs[0];
    processed_acc_y_g = *accel_outputs[1];
    processed_acc_z_g = *accel_outputs[2];
}


