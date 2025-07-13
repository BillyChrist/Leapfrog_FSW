/*
 * altimeter.h
 *
 *  Created on: Mar 7, 2025
 *      Author: Billy Christ
 */

#ifndef INC_ALTIMETER_H_
#define INC_ALTIMETER_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>


typedef enum {
    ALTIMETER_SHORT_DISTANCE_MODE,
    ALTIMETER_LONG_DISTANCE_MODE
} AltimeterDistanceMode;

typedef struct {
    uint16_t altitude;                      // Altitude (in centimeters)
    uint16_t signal_strength;               // Signal strength
    AltimeterDistanceMode distance_mode;    // Distance mode (short/long)
} AltimeterData;

// Extern declaration for global access
extern AltimeterData latestData;
//extern volatile bool lidarStatusReadPending;

// External declarations for LIDAR status flags
extern volatile bool lidarMeasurementInProgress;
extern volatile bool lidarDistanceReadInProgress;
extern volatile bool lidarDataReady;

// Function prototypes
void altimeterInit(I2C_HandleTypeDef* i2c, AltimeterDistanceMode mode);
void altimeterGetLatestData(AltimeterData* altimeterData);
void altimeterReadFrame(void);
HAL_StatusTypeDef altimeterCheckI2CReady(void);



#endif /* INC_ALTIMETER_H_ */
