/*
 * altimeter.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Billy Christ
 */

#include "altimeter.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "queue.h"
#include "main.h"
#include "heartbeat.h"

// Constants and header values
#define LIDAR_I2C_ADDRESS      0x62  // Default I2C address
#define LIDAR_ACQ_COMMAND      0x00  // Register to initiate a measurement
#define LIDAR_MEASURE_VALUE    0x04  // Value to initiate a measurement
#define LIDAR_STATUS_REG       0x01  // Status register
#define LIDAR_DIST_REG         0x8F  // Distance register (high byte)

// Local Data
AltimeterData latestData;
static I2C_HandleTypeDef* i2cHandle;

// Temporary buffer for storing read data
static uint8_t distanceBuffer[2];
static uint8_t status;  // Temporary status buffer for reading LIDAR status register

// Flags to track I2C status
volatile bool lidarMeasurementInProgress = false;
volatile bool lidarDistanceReadInProgress = false;
volatile bool lidarDataReady = false;
volatile HAL_StatusTypeDef lidarLastStatus;

// Callback function prototypes
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);




//void altimeterCalibration(){
//	// TODO Altimeter Calibration. Check values from alt_imu_coupling.c and sum over 100 samples.
//	// (similar to processIMU.c imu calibration)
//}



/**
 * @brief Test if LIDAR device is ready on I2C
 * @return HAL_OK if device is ready, HAL_ERROR otherwise
 */
HAL_StatusTypeDef altimeterCheckI2CReady(void) {
    if (i2cHandle == NULL) {
        printf("I2C Handle is NULL. Altimeter not initialized.\r\n");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(i2cHandle, LIDAR_I2C_ADDRESS << 1, 3, 100);
    if (ret != HAL_OK) {
        printf("LIDAR Device Not Ready on I2C!\r\n");
    } else {
        printf("LIDAR Device is Ready!\r\n");
    }
    return ret;
}

/**
 * @brief Initialize altimeter module using I2C
 */
void altimeterInit(I2C_HandleTypeDef* i2c, AltimeterDistanceMode mode) {
    i2cHandle = i2c;

    HAL_Delay(10);  // Short delay to ensure completion

    // Configure for standard acquisition mode
    uint8_t sigCount = 0x80;
    uint8_t acqConfig = 0x00;
    uint8_t distanceModeValue;

    if (mode == ALTIMETER_SHORT_DISTANCE_MODE) {
        distanceModeValue = 0x03;  // Short distance mode (low sensitivity, high precision)
        latestData.distance_mode = ALTIMETER_SHORT_DISTANCE_MODE;
        printf("\rInitializing LIDAR in SHORT Distance Mode.\r\n");
    } else {
        distanceModeValue = 0x01;  // Long distance mode (higher sensitivity, longer range)
        latestData.distance_mode = ALTIMETER_LONG_DISTANCE_MODE;
        printf("\rInitializing LIDAR in LONG Distance Mode.\r\n");
    }

    printf("\rConfiguring LIDAR Settings...\r\n");

    // Write sigCount
    if (HAL_I2C_Mem_Write(i2cHandle, LIDAR_I2C_ADDRESS << 1, 0x02, 1, &sigCount, 1, 100) != HAL_OK) {
        printf("\rI2C Write Failed for sigCount.\r\n");
        Error_Handler();
    }
    HAL_Delay(10);  // Short delay to ensure completion

    // Write acqConfig
    if (HAL_I2C_Mem_Write(i2cHandle, LIDAR_I2C_ADDRESS << 1, 0x04, 1, &acqConfig, 1, 100) != HAL_OK) {
        printf("\rI2C Write Failed for acqConfig.\r\n");
        Error_Handler();
    }
    HAL_Delay(10);  // Short delay to ensure completion

    // Write threshold bypass register to set distance mode (0x1C register)
    if (HAL_I2C_Mem_Write(i2cHandle, LIDAR_I2C_ADDRESS << 1, 0x1C, 1, &distanceModeValue, 1, 100) != HAL_OK) {
        printf("\rI2C Write Failed for Distance Mode Configuration.\r\n");
        Error_Handler();
    }
    HAL_Delay(10);  // Short delay to ensure completion

    printf("\rLIDAR Initialization Complete.\r\n");
}

/**
 * @brief Read distance data from LIDAR Lite v3 using I2C
 */
void altimeterReadFrame(void) {
    uint8_t cmd = LIDAR_MEASURE_VALUE;
    lidarMeasurementInProgress = true;
    lidarDistanceReadInProgress = false;
    lidarDataReady = false;

    if (HAL_I2C_Mem_Write_IT(i2cHandle, LIDAR_I2C_ADDRESS << 1, LIDAR_ACQ_COMMAND, 1, &cmd, 1) != HAL_OK) {
        printf("LIDAR Measurement Initiation Failed!\r\n");
    } else {
        osDelay(01);  // Allowing time for LIDAR to start processing measurement
    }
}

/**
 * @brief Get latest altimeter data
 *
 * @param altimeterData Pointer to AltimeterData structure
 */
void altimeterGetLatestData(AltimeterData* altimeterData) {
    *altimeterData = latestData;
}

/**
 * @brief I2C Transmission Complete Callback
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == i2cHandle) {
        lidarMeasurementInProgress = false;
//        osDelay(10);  // Allow LIDAR time to complete measurement
        if (HAL_I2C_Mem_Read_IT(i2cHandle, LIDAR_I2C_ADDRESS << 1, LIDAR_STATUS_REG, 1, &status, 1) != HAL_OK) {
            printf("Failed to read LIDAR status!\r\n");
        }
    }
}

/**
 * @brief I2C Reception Complete Callback
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == i2cHandle) {
        if (lidarDistanceReadInProgress) {
            latestData.altitude = (distanceBuffer[0] << 8) | distanceBuffer[1];
//            stm32Response.altitude = (int16_t)latestData.altitude;
            lidarDataReady = true;
            lidarDistanceReadInProgress = false;
        } else {
            // Skip busy check, assume timing is managed correctly
            if (HAL_I2C_Mem_Read_IT(i2cHandle, LIDAR_I2C_ADDRESS << 1, LIDAR_DIST_REG, 1, distanceBuffer, 2) == HAL_OK) {
                lidarDistanceReadInProgress = true;
            } else {
                printf("LIDAR Distance Read Initiation Failed!\r\n");
            }
        }
    }
}

/**
 * @brief I2C Error Callback
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == i2cHandle) {
        printf("I2C Communication Error! Resetting LIDAR flags.\r\n");
        lidarMeasurementInProgress = false;
        lidarDistanceReadInProgress = false;
        lidarDataReady = false;
    }
}
