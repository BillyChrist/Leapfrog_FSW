/* ******************************************************************************
 * @file   : gps.c
 *
 * @brief  : This module handles GPS sensor data. 
 * 
 *  Created on: Mar 7, 2025
 *      Author: Billy Christ
 *  ******************************************************************************
 */

// TODO move sensor to raspberry pi, or include redundant GPS sensor data and fusion between controllers

#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static UART_HandleTypeDef *gpsUartHandle;
static uint8_t gpsBuf[GPS_PACKET_LEN];
GPS_Data latestGPSdata;

// GPS configuration constants
#define GPS_DATA_TIMEOUT_MS 5000        // GPS data timeout in milliseconds
#define GPS_ALTITUDE_TOLERANCE_M 10.0   // Altitude cross-check tolerance in meters
#define GPS_VELOCITY_SMOOTHING_FACTOR 0.1  // Velocity smoothing factor (0.0-1.0)
#define GPS_POSITION_SMOOTHING_FACTOR 0.05 // Position smoothing factor (0.0-1.0)

// Earth radius constants for distance calculations
#define EARTH_RADIUS_M 6371000.0        // Earth radius in meters
#define DEG_TO_RAD (M_PI / 180.0)       // Degrees to radians conversion

// Previous position for velocity calculation
static float prev_latitude = 0.0;
static float prev_longitude = 0.0;
static float prev_altitude = 0.0;
static uint32_t prev_timestamp_ms = 0;


// Initialize UART
void gpsInit(UART_HandleTypeDef *huart) {
    gpsUartHandle = huart;
    
    // Initialize GPS data structure
    memset(&latestGPSdata, 0, sizeof(GPS_Data));
    latestGPSdata.data_valid = false;
    latestGPSdata.position_valid = false;
    latestGPSdata.velocity_valid = false;
    latestGPSdata.altitude_crosscheck_valid = false;
    
    // Initialize previous position tracking
    prev_latitude = 0.0;
    prev_longitude = 0.0;
    prev_altitude = 0.0;
    prev_timestamp_ms = 0;
    
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
    
    // Update timestamp and data validity
    gpsData->last_update_ms = HAL_GetTick();
    gpsData->data_valid = (gpsData->fix_status > 0);
    gpsData->position_valid = (gpsData->fix_status > 0 && 
                              gpsData->latitude != 0.0 && 
                              gpsData->longitude != 0.0);
    
    // Calculate velocity from position changes
    gpsUpdateVelocity(gpsData);
    
    // Calculate position error if target is set
    if (gpsData->target_latitude != 0.0 && gpsData->target_longitude != 0.0) {
        gpsCalculatePositionError(gpsData);
    }
    
    // Update drift compensation
    gpsUpdateDriftCompensation(gpsData);
}

/**
 * @brief Calculate velocity from position changes
 * @param gpsData: GPS data structure to update
 */
void gpsUpdateVelocity(GPS_Data *gpsData) {
    uint32_t current_time = HAL_GetTick();
    
    // Check if we have previous position data
    if (prev_timestamp_ms != 0 && prev_latitude != 0.0 && prev_longitude != 0.0) {
        float dt = (current_time - prev_timestamp_ms) / 1000.0f; // Convert to seconds
        
        if (dt > 0.1f && dt < 10.0f) { // Valid time difference (0.1s to 10s)
            // Calculate distance in meters using Haversine formula
            float lat1_rad = prev_latitude * DEG_TO_RAD;
            float lat2_rad = gpsData->latitude * DEG_TO_RAD;
            float dlat_rad = (gpsData->latitude - prev_latitude) * DEG_TO_RAD;
            float dlon_rad = (gpsData->longitude - prev_longitude) * DEG_TO_RAD;
            
            float a = sinf(dlat_rad/2) * sinf(dlat_rad/2) + 
                     cosf(lat1_rad) * cosf(lat2_rad) * 
                     sinf(dlon_rad/2) * sinf(dlon_rad/2);
            float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
            float distance_m = EARTH_RADIUS_M * c;
            
            // Calculate bearing
            float y = sinf(dlon_rad) * cosf(lat2_rad);
            float x = cosf(lat1_rad) * sinf(lat2_rad) - 
                     sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon_rad);
            float bearing_rad = atan2f(y, x);
            float bearing_deg = bearing_rad * 180.0f / M_PI;
            if (bearing_deg < 0) bearing_deg += 360.0f;
            
            // Calculate velocity components
            float velocity_ms = distance_m / dt;
            float velocity_north = velocity_ms * cosf(bearing_rad);
            float velocity_east = velocity_ms * sinf(bearing_rad);
            float velocity_up = (gpsData->altitude - prev_altitude) / dt;
            
            // Apply smoothing to velocity
            gpsData->velocity_north_ms = GPS_VELOCITY_SMOOTHING_FACTOR * velocity_north + 
                                       (1.0f - GPS_VELOCITY_SMOOTHING_FACTOR) * gpsData->velocity_north_ms;
            gpsData->velocity_east_ms = GPS_VELOCITY_SMOOTHING_FACTOR * velocity_east + 
                                      (1.0f - GPS_VELOCITY_SMOOTHING_FACTOR) * gpsData->velocity_east_ms;
            gpsData->velocity_up_ms = GPS_VELOCITY_SMOOTHING_FACTOR * velocity_up + 
                                    (1.0f - GPS_VELOCITY_SMOOTHING_FACTOR) * gpsData->velocity_up_ms;
            
            gpsData->speed_ms = sqrtf(gpsData->velocity_north_ms * gpsData->velocity_north_ms + 
                                     gpsData->velocity_east_ms * gpsData->velocity_east_ms);
            gpsData->heading_deg = bearing_deg;
            gpsData->velocity_valid = true;
        }
    }
    
    // Update previous position data
    prev_latitude = gpsData->latitude;
    prev_longitude = gpsData->longitude;
    prev_altitude = gpsData->altitude;
    prev_timestamp_ms = current_time;
}

/**
 * @brief Calculate position error from target
 * @param gpsData: GPS data structure to update
 */
void gpsCalculatePositionError(GPS_Data *gpsData) {
    if (!gpsData->position_valid || gpsData->target_latitude == 0.0 || gpsData->target_longitude == 0.0) {
        return;
    }
    
    // Calculate distance to target using Haversine formula
    float lat1_rad = gpsData->latitude * DEG_TO_RAD;
    float lat2_rad = gpsData->target_latitude * DEG_TO_RAD;
    float dlat_rad = (gpsData->target_latitude - gpsData->latitude) * DEG_TO_RAD;
    float dlon_rad = (gpsData->target_longitude - gpsData->longitude) * DEG_TO_RAD;
    
    float a = sinf(dlat_rad/2) * sinf(dlat_rad/2) + 
             cosf(lat1_rad) * cosf(lat2_rad) * 
             sinf(dlon_rad/2) * sinf(dlon_rad/2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    float distance_m = EARTH_RADIUS_M * c;
    
    // Calculate bearing to target
    float y = sinf(dlon_rad) * cosf(lat2_rad);
    float x = cosf(lat1_rad) * sinf(lat2_rad) - 
             sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon_rad);
    float bearing_rad = atan2f(y, x);
    
    // Convert to north/east error
    gpsData->position_error_north_m = distance_m * cosf(bearing_rad);
    gpsData->position_error_east_m = distance_m * sinf(bearing_rad);
}

/**
 * @brief Set target position for navigation
 * @param latitude: Target latitude in degrees
 * @param longitude: Target longitude in degrees
 */
void gpsSetTargetPosition(float latitude, float longitude) {
    latestGPSdata.target_latitude = latitude;
    latestGPSdata.target_longitude = longitude;
    
    // Recalculate position error
    gpsCalculatePositionError(&latestGPSdata);
}

/**
 * @brief Update drift compensation based on position error
 * @param gpsData: GPS data structure to update
 */
void gpsUpdateDriftCompensation(GPS_Data *gpsData) {
    if (!gpsData->position_valid) {
        gpsData->drift_velocity_north_ms = 0.0f;
        gpsData->drift_velocity_east_ms = 0.0f;
        return;
    }
    
    // Simple proportional drift compensation
    // This could be enhanced with PID control
    float drift_gain = 0.1f; // Adjust based on system response
    
    gpsData->drift_velocity_north_ms = -drift_gain * gpsData->position_error_north_m;
    gpsData->drift_velocity_east_ms = -drift_gain * gpsData->position_error_east_m;
}

/**
 * @brief Cross-check GPS altitude with altimeter
 * @param gpsData: GPS data structure to update
 * @param altimeter_altitude_m: Altitude from altimeter in meters
 */
void gpsCrossCheckAltitude(GPS_Data *gpsData, float altimeter_altitude_m) {
    gpsData->altitude_altimeter_m = altimeter_altitude_m;
    gpsData->altitude_difference_m = gpsData->altitude - altimeter_altitude_m;
    
    // Check if altitude difference is within tolerance
    gpsData->altitude_crosscheck_valid = (fabsf(gpsData->altitude_difference_m) < GPS_ALTITUDE_TOLERANCE_M);
}

/**
 * @brief Check if GPS data is valid and recent
 * @param gpsData: GPS data structure to check
 * @return true if data is valid and recent
 */
bool gpsIsDataValid(GPS_Data *gpsData) {
    uint32_t current_time = HAL_GetTick();
    gpsData->data_age_ms = current_time - gpsData->last_update_ms;
    
    return (gpsData->data_valid && 
            gpsData->data_age_ms < GPS_DATA_TIMEOUT_MS && 
            gpsData->position_valid);
}

/**
 * @brief Get drift compensation velocities
 * @param north_compensation_ms: North drift compensation velocity (output)
 * @param east_compensation_ms: East drift compensation velocity (output)
 */
void gpsGetDriftCompensation(float *north_compensation_ms, float *east_compensation_ms) {
    if (gpsIsDataValid(&latestGPSdata)) {
        *north_compensation_ms = latestGPSdata.drift_velocity_north_ms;
        *east_compensation_ms = latestGPSdata.drift_velocity_east_ms;
    } else {
        *north_compensation_ms = 0.0f;
        *east_compensation_ms = 0.0f;
    }
}

/**
 * @brief Get position error for TVC integration
 * @param north_error_m: North position error in meters (output)
 * @param east_error_m: East position error in meters (output)
 */
void gpsGetPositionError(float *north_error_m, float *east_error_m) {
    if (gpsIsDataValid(&latestGPSdata)) {
        *north_error_m = latestGPSdata.position_error_north_m;
        *east_error_m = latestGPSdata.position_error_east_m;
    } else {
        *north_error_m = 0.0f;
        *east_error_m = 0.0f;
    }
}

/**
 * @brief Get velocity for TVC integration
 * @param north_velocity_ms: North velocity in m/s (output)
 * @param east_velocity_ms: East velocity in m/s (output)
 */
void gpsGetVelocity(float *north_velocity_ms, float *east_velocity_ms) {
    if (gpsIsDataValid(&latestGPSdata) && latestGPSdata.velocity_valid) {
        *north_velocity_ms = latestGPSdata.velocity_north_ms;
        *east_velocity_ms = latestGPSdata.velocity_east_ms;
    } else {
        *north_velocity_ms = 0.0f;
        *east_velocity_ms = 0.0f;
    }
}

