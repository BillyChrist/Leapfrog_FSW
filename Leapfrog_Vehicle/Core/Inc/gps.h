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
    // Position data
    float latitude;
    float longitude;
    float altitude;
    uint8_t fix_status; // 0 = No fix, 1 = GPS fix, 2 = DGPS fix
    
    // Velocity data (calculated from position changes)
    float velocity_north_ms;    // North velocity in m/s
    float velocity_east_ms;     // East velocity in m/s
    float velocity_up_ms;       // Vertical velocity in m/s
    float speed_ms;             // Ground speed in m/s
    float heading_deg;          // Ground track heading in degrees
    
    // Position tracking for drift compensation
    float target_latitude;      // Target position for navigation
    float target_longitude;     // Target position for navigation
    float position_error_north_m;  // North position error in meters
    float position_error_east_m;   // East position error in meters
    float drift_velocity_north_ms; // Drift velocity north in m/s
    float drift_velocity_east_ms;  // Drift velocity east in m/s
    
    // Data quality and timing
    uint32_t last_update_ms;    // Timestamp of last GPS update
    uint32_t data_age_ms;       // Age of current GPS data
    bool data_valid;            // True if GPS data is recent and valid
    bool position_valid;        // True if position is valid for navigation
    bool velocity_valid;        // True if velocity data is valid
    
    // Altitude cross-check with altimeter
    float altitude_altimeter_m; // Altitude from altimeter for cross-check
    float altitude_difference_m; // Difference between GPS and altimeter altitude
    bool altitude_crosscheck_valid; // True if altitude cross-check is within tolerance
} GPS_Data;
extern GPS_Data latestGPSdata;

// Function prototypes
void gpsInit(UART_HandleTypeDef *huart);
void gpsUARTRXCallback(uint8_t *bufferGPS);
void parseGPSData(uint8_t *buffer, GPS_Data *gpsData);

// Enhanced GPS functionality
void gpsUpdateVelocity(GPS_Data *gpsData);
void gpsCalculatePositionError(GPS_Data *gpsData);
void gpsSetTargetPosition(float latitude, float longitude);
void gpsUpdateDriftCompensation(GPS_Data *gpsData);
void gpsCrossCheckAltitude(GPS_Data *gpsData, float altimeter_altitude_m);
bool gpsIsDataValid(GPS_Data *gpsData);
void gpsGetDriftCompensation(float *north_compensation_ms, float *east_compensation_ms);

// TVC integration functions
void gpsGetPositionError(float *north_error_m, float *east_error_m);
void gpsGetVelocity(float *north_velocity_ms, float *east_velocity_ms);


#endif /* INC_GPS_H_ */
