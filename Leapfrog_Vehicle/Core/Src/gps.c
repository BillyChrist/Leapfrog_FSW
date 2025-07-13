/*
 * gps.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Billy Christ
 */


#include "gps.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *gpsUartHandle;
static uint8_t gpsBuf[GPS_PACKET_LEN];
GPS_Data latestGPSdata;


// Initialize UART
void gpsInit(UART_HandleTypeDef *huart) {
    gpsUartHandle = huart;
    HAL_UART_Receive_IT(gpsUartHandle, gpsBuf, GPS_PACKET_LEN);
}

// UART RX Callback
void gpsUARTRXCallback(uint8_t *bufferGPS) {
    if (bufferGPS[0] == '$') {  // Check for NMEA start character
        if (memcmp(bufferGPS + 1, "GPGGA", 5) == 0) {
            parseGPSData(bufferGPS, &latestGPSdata);
        }
    }
    // Restart UART reception
    HAL_UART_Receive_IT(gpsUartHandle, gpsBuf, GPS_PACKET_LEN);
}

// Parse NMEA GPGGA
void parseGPSData(uint8_t *buffer, GPS_Data *gpsData) {
    char *token;
    char data[GPS_PACKET_LEN];
    strncpy(data, (char *)buffer, GPS_PACKET_LEN);

    token = strtok(data, ","); // Start of sentence
    token = strtok(NULL, ","); // Time (ignored)

    token = strtok(NULL, ","); // Latitude
    if (token) gpsData->latitude = atof(token);

    token = strtok(NULL, ","); // N/S Indicator (adjust sign)
    if (token && token[0] == 'S') gpsData->latitude = -gpsData->latitude;

    token = strtok(NULL, ","); // Longitude
    if (token) gpsData->longitude = atof(token);

    token = strtok(NULL, ","); // E/W Indicator (adjust sign)
    if (token && token[0] == 'W') gpsData->longitude = -gpsData->longitude;

    token = strtok(NULL, ","); // Fix Status
    if (token) gpsData->fix_status = atoi(token);

    token = strtok(NULL, ","); // Number of satellites (ignored)

    token = strtok(NULL, ","); // Altitude
    if (token) gpsData->altitude = atof(token);
}

