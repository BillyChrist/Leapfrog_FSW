/*
 * ACS_PID.h
 *
 *  Created on: Jul 18, 2024
 *      Author: BillyChrist
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "processIMU.h" // Ensures access to IMU variables like angular velocity if needed

extern void ACS_PID_Control(void);

extern float yaw_pointing;

/* PID variables */
extern float error_pitch;
extern float error_roll;
extern float error_yaw;

extern float output_pitch;
extern float output_roll;
extern float output_yaw;

extern float integral_pitch;
extern float integral_roll;
extern float integral_yaw;

extern float prev_error_pitch;
extern float prev_error_roll;
extern float prev_error_yaw;

extern float PR1_output;
extern float PR2_output;
extern float PR3_output;
extern float PR4_output;
extern float Y1_output;
extern float Y2_output;

extern float P_pitch;
extern float P_roll;
extern float P_yaw;

extern float I_pitch;
extern float I_roll;
extern float I_yaw;

extern float D_pitch;
extern float D_roll;
extern float D_yaw;

extern float PID_pitch;
extern float PID_roll;
extern float PID_yaw;

extern float correction_pitch;
extern float correction_roll;
extern float correction_yaw;

/* PID constants */
extern float Kp_rollpitch;
extern float Ki_rollpitch;
extern float Kd_rollpitch;

extern float Kp_yaw;
extern float Ki_yaw;
extern float Kd_yaw;

extern float idle_rollpitch;
extern float idle_yaw;

extern uint32_t lastUpdate;

#endif /* INC_PID_H_ */
