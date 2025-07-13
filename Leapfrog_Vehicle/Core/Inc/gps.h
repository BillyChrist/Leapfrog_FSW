/*
 * gps.h
 *
 *  Created on: Mar 7, 2025
 *      Author: Billy Christ
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_


#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_PACKET_LEN 128  // Adjust based on GPS module output size

// Struct to store parsed GPS data
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint8_t fix_status; // 0 = No fix, 1 = GPS fix, 2 = DGPS fix
} GPS_Data;
extern GPS_Data latestGPSdata;

// Function prototypes
void gpsInit(UART_HandleTypeDef *huart);
void gpsUARTRXCallback(uint8_t *bufferGPS);
void parseGPSData(uint8_t *buffer, GPS_Data *gpsData);


#endif /* INC_GPS_H_ */
