/*
 * tvc.h
 *
 *  Created on: Jul 18, 2024
 *      Author: BillyChrist
 */

#ifndef __TVC_H
#define __TVC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern void Extend(int axis);
extern void Retract(int axis);
extern void Hold(int axis);
extern void TVC_Controller(void* argument);
extern void TVC_Manual(void);
extern void HomePosition(int axis);
extern void Test_TVC();


//extern osMessageQueueId_t tvcimuQueueHandle;
extern uint32_t adcValues[4];
extern SubsystemState tvcState; // State can be SystemDisabled, SystemManual, or SystemAutomatic
extern float tvcManualSetpoint1;
extern float tvcManualSetpoint2;
extern float tvcVal1_angular;
extern float tvcVal2_angular;

typedef struct { // TODO Compile data in this packet or remove! This is a placeholder for the TVC queue creation!
    uint16_t adcValues[4];
} TVC_Data;

// Extern declaration for global access
extern TVC_Data latestTVC_Data;


#ifdef __cplusplus
}
#endif
#endif
