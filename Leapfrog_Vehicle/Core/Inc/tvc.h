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
extern TVC_State tvcState; // State can be SystemTVC_Disable, SystemTVC_Manual, or SystemTVC_Automatic
extern float tvcManualSetpoint1;
extern float tvcManualSetpoint2;
extern float tvcVal1_angular;
extern float tvcVal2_angular;

// IMU data - Vehicle reference frame (defined in ProcessIMU.c)
extern float tvc_roll_deg;                   // Current vehicle roll for TVC (degrees)
extern float tvc_pitch_deg;                  // Current vehicle pitch for TVC (degrees)
extern float tvc_yaw_deg;                    // Current vehicle yaw for TVC (degrees)
extern float tvc_angvel_x_degs;              // Angular velocity X (roll rate) for TVC (deg/s)
extern float tvc_angvel_y_degs;              // Angular velocity Y (pitch rate) for TVC (deg/s)
extern float tvc_angvel_z_degs;              // Angular velocity Z (yaw rate) for TVC (deg/s)
extern float tvc_acc_x_g;                    // Acceleration X for TVC (g)
extern float tvc_acc_y_g;                    // Acceleration Y for TVC (g)
extern float tvc_acc_z_g;                    // Acceleration Z for TVC (g)

// Engine data
extern float throttle;                       // Current engine throttle percentage (0-100)

typedef struct { // TODO Compile data in this packet or remove! This is a placeholder for the TVC queue creation!
    uint16_t adcValues[4];
} TVC_Data;

// Extern declaration for global access
extern TVC_Data latestTVC_Data;


#ifdef __cplusplus
}
#endif
#endif
