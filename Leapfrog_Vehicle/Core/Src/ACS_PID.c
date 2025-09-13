/* ******************************************************************************
 * @file   : ACS_PID.c
 *
 * @brief  : This code reads the IMU data, performs PID calculations, and outputs updated ACS power variables.
 * 
 *  Created on: Aug 14, 2024
 *      Author: BillyChrist
 *  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ACS_PID.h"
#include "processIMU.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// TODO: use angvel or ang_accel to assist with critical damping, and counter over-shooting


// Consider converting to arrays
// ex: PID[] = {1.200, 0.000, 0.050}
// current[P, R, Y] = {.f, .f, .f}
// ... etc

/* ~~~~~~~~~~~~~ PID constants ~~~~~~~~~~~~~ */
	// Constants for roll & pitch fans
	float Kp_rollpitch = 2.000f;
	float Ki_rollpitch = 1.200f;
	float Kd_rollpitch = 0.900f;

	// Constants for yaw fans
	float Kp_yaw = 0.500f;
	float Ki_yaw = 0.200f;
	float Kd_yaw = 0.900f;

float yaw_pointing = 0.00f;

// Determine the maximum PID output value
float max_pid_output = 95.0f;              // Adjust based on output range
float min_pid_output = 60.0f;

// Integral Clamping values
// Explanation in line with code
float pitchroll_integral_clamp_max = (19.5/(.1389*3));
float pitchroll_integral_clamp_min = (-14.5/(.1389*3));

float yaw_integral_clamp_max = (19.5 / 0.1389);
float yaw_integral_clamp_min = (-14.5 / 0.1389);

// Idle speed
float idle_rollpitch = 75.0f;
float idle_yaw = 75.0f;

// TODO These terms were replaced - Delete this after testing system
//float current_roll_deg;
//float current_pitch_deg;
//float current_yaw_deg;

//float current_angvel_x_degs;
//float current_angvel_y_degs;


/* ~~~~~~~~~~~~~ Variables~~~~~~~~~~~~~ */
float error_pitch;
float error_roll;
float error_yaw;

float prev_error_pitch;
float prev_error_roll;
float prev_error_yaw;

float P_pitch;
float P_roll;
float P_yaw;

float integral_pitch;
float integral_roll;
float integral_yaw;

float I_pitch;
float I_roll;
float I_yaw;

float derivative_pitch;
float derivative_roll;
float derivative_yaw;

float D_pitch;
float D_roll;
float D_yaw;

float PID_pitch;
float PID_roll;
float PID_yaw;

float correction_pitch;
float correction_roll;
float correction_yaw;

float output_pitch;
float output_roll;
float output_yaw;

float PR1_output;
float PR2_output;
float PR3_output;
float PR4_output;
float Y1_output;
float Y2_output;

uint32_t lastUpdate = 0;


/* ~~~~~~~~~~~~~  MAIN LOOP ~~~~~~~~~~~~~  */

void ACS_PID_Control(void) {
    uint32_t currentTime = HAL_GetTick(); 			   // Current time in ms
    float dt = (currentTime - lastUpdate) / 1000.0;    // Time difference in seconds
    if (dt < 0.1) return; 							   // Return if less than 100 ms have passed

    lastUpdate = currentTime;

    // Define desired setpoints (0.0 = level)
    float setpoint_pitch = 0.0f;
    float setpoint_roll  = 0.0f;
    float setpoint_yaw = yaw_pointing + 0.0f;

    /* Errors */
    // Calculate errors based on setpoints (IMU data already corrected for wrap-around)
    error_pitch = setpoint_pitch + processed_pitch_deg;
    error_roll  = setpoint_roll  + processed_roll_deg;
    error_yaw   = setpoint_yaw   + processed_yaw_deg;

    /* PID Calculations: PID = e*Kp + ∫e*Ki + Δe*Kd */

    // Proportional terms
    P_pitch = Kp_rollpitch * error_pitch;
    P_roll  = Kp_rollpitch * error_roll;
    P_yaw   = Kp_yaw * error_yaw;

    // Derivative terms
    derivative_pitch = (error_pitch - prev_error_pitch) / dt;
    derivative_roll  = (error_roll  - prev_error_roll)  / dt;
    derivative_yaw   = (error_yaw   - prev_error_yaw)   / dt;

    D_pitch = Kd_rollpitch * derivative_pitch;
    D_roll  = Kd_rollpitch * derivative_roll;
    D_yaw   = Kd_yaw * derivative_yaw;

    /* Integral Term Calculations & Windup Clamping
    Clamps to just below min and max values in output.
    Idle is 75 (50%), so -14.5 brings output to just above min output (60)
    19.5 brings output to just below maximum output (95)
    If the I terms are within the bounds, the integrator is allowed to continue to sum the errors
    If the I terms are not within the bounds, the integrator is not allowed to sum the errors in
    	order to prevent integral saturation
    */

    	// Windup clamp for pitch I-term, only allows integral term to sum if fans are not saturated
    	if (((I_pitch < pitchroll_integral_clamp_max) && (I_pitch > pitchroll_integral_clamp_min))){// || ((prev_error_pitch > 0) && (error_pitch < 0)) || ((prev_error_pitch < 0) && (error_pitch > 0))){
    		integral_pitch += error_pitch * dt;
    		I_pitch = Ki_rollpitch * integral_pitch;
    	}
    	else if ((I_pitch >= pitchroll_integral_clamp_max)) {
    		if((prev_error_pitch > 0) && (error_pitch > 0))
    	        I_pitch = pitchroll_integral_clamp_max;
    		else{
    			integral_pitch += error_pitch * dt;
				I_pitch = Ki_rollpitch * integral_pitch;
    		}
    	}
    	else if (I_pitch <= pitchroll_integral_clamp_min) {
    		if((prev_error_pitch < 0) && (error_pitch < 0))
				I_pitch = pitchroll_integral_clamp_min;
			else{
				integral_pitch += error_pitch * dt;
				I_pitch = Ki_rollpitch * integral_pitch;
			}
    	}

    	// Windup clamp for roll I-term, only allows integral terms to sum if fans are not saturated
    	if (((I_roll < pitchroll_integral_clamp_max) && (I_roll > pitchroll_integral_clamp_min))){
    	    		integral_roll += error_roll * dt;
    	    		I_roll = Ki_rollpitch * integral_roll;
    	}
    	else if ((I_roll >= pitchroll_integral_clamp_max)) {
    		if((prev_error_roll > 0) && (error_roll > 0))
    	    	I_roll = pitchroll_integral_clamp_max;
    	    else{
    	    	integral_roll += error_roll * dt;
    			I_roll = Ki_rollpitch * integral_roll;
    	    }
    	}
    	else if (I_roll <= pitchroll_integral_clamp_min) {
    	    if((prev_error_roll < 0) && (error_roll < 0))
    			I_roll = pitchroll_integral_clamp_min;
    		else{
    			integral_roll += error_roll * dt;
    			I_roll = Ki_rollpitch * integral_roll;
    		}
    	}

     // Yaw Windup Clamping, only allows integral terms to sum if yaw fans are not saturated
     if (((I_yaw < yaw_integral_clamp_max) && (I_yaw > yaw_integral_clamp_min))){
         		integral_yaw += error_yaw * dt;
         		I_yaw = Ki_yaw * integral_yaw;
     }
     else if ((I_yaw >= yaw_integral_clamp_max)) {
         	if((prev_error_yaw > 0) && (error_yaw > 0))
         	    I_yaw = yaw_integral_clamp_max;
         	else{
         		integral_yaw += error_yaw * dt;
     			I_yaw = Ki_yaw * integral_yaw;
         	}
     }
     else if (I_yaw <= yaw_integral_clamp_min) {
         	if((prev_error_yaw < 0) && (error_yaw < 0))
     			I_yaw = yaw_integral_clamp_min;
     		else{
     			integral_yaw += error_yaw * dt;
     			I_yaw = Ki_yaw * integral_yaw;
     		}
     }

// TODO Add overshoot ramp-up, (from ang_vel)

    // If angle is approaching zero, flip integral sign at 0?
    // "Critical Damping" - anticipate angular momentum

    // Implement a "Deadzone" to prevent rapid reactions at equilibrium

  // Example Ramp-up logic
  //       if (target_CCR_pitch > CCR_pitch) {
  //           CCR_pitch += ramp_up_rate * dt;
  //           if (CCR_pitch > target_CCR_pitch) CCR_pitch = target_CCR_pitch;
  //       } else if (target_CCR_pitch < CCR_pitch) {
  //           CCR_pitch -= ramp_down_rate * dt;
  //           if (CCR_pitch < target_CCR_pitch) CCR_pitch = target_CCR_pitch;
  //       }
  //
  //       if (target_CCR_roll > CCR_roll) {
  //           CCR_roll += ramp_up_rate * dt;
  //           if (CCR_roll > target_CCR_roll) CCR_roll = target_CCR_roll;
  //       } else if (target_CCR_roll < CCR_roll) {
  //           CCR_roll -= ramp_down_rate * dt;
  //           if (CCR_roll < target_CCR_roll) CCR_roll = target_CCR_roll;
  //       }
  //
  //       if (target_CCR_yaw > CCR_yaw) {
  //           CCR_yaw += ramp_up_rate * dt;
  //           if (CCR_yaw > target_CCR_yaw) CCR_yaw = target_CCR_yaw;
  //       } else if (target_CCR_yaw < CCR_yaw) {
  //           CCR_yaw -= ramp_down_rate * dt;
  //           if (CCR_yaw < target_CCR_yaw) CCR_yaw = target_CCR_yaw;
  //       }



    // Compute total PID outputs
    PID_pitch = P_pitch + I_pitch + D_pitch;
    PID_roll  = P_roll + I_roll + D_roll;
    PID_yaw   = P_yaw + I_yaw + D_yaw;

    // Compute CCR values (all are additions to the idle speed)
    // TODO verify calculations here: 25/180 [max deviation / max angle]
    //float correctionFactor = 25/180; // ~= 0.138888...

    correction_pitch = PID_pitch * 0.138888;
    correction_roll  = PID_roll  * 0.138888;
    correction_yaw   = PID_yaw   * 0.138888;

/*
       Previous Config:
       *      (1)				x
       *       |              z ↑
       * (4)-- x --(2)         \|
       *       |                +---→ y
       *      (3)

       Updated Config:
       *    (1) (2)             x
       *      \ /             z ↑
       *       X               \|
       *      / \               +---→ y
       *    (4) (3)
       * Motors 1 & 3 run clockwise, Motors 2 & 4 run counter-clockwise.
       * Front fans and rear fans coupled for pitch. Fan 1&4, and 2&3 coupled for roll
*/

    	// Configure PID Outputs for each fan
    	PR1_output = idle_rollpitch + correction_pitch - correction_roll;  // (1) Front Left
    	PR2_output = idle_rollpitch + correction_pitch + correction_roll;  // (2) Front Right
    	PR3_output = idle_rollpitch - correction_pitch + correction_roll;  // (3) Back Right
    	PR4_output = idle_rollpitch - correction_pitch - correction_roll;  // (4) Back Left
    	Y1_output  = idle_yaw - correction_yaw;
    	Y2_output  = idle_yaw + correction_yaw;

    	// Constrain each output within min/max limits
    	PR1_output = (PR1_output < min_pid_output) ? min_pid_output : (PR1_output > max_pid_output) ? max_pid_output : PR1_output;
    	PR2_output = (PR2_output < min_pid_output) ? min_pid_output : (PR2_output > max_pid_output) ? max_pid_output : PR2_output;
    	PR3_output = (PR3_output < min_pid_output) ? min_pid_output : (PR3_output > max_pid_output) ? max_pid_output : PR3_output;
    	PR4_output = (PR4_output < min_pid_output) ? min_pid_output : (PR4_output > max_pid_output) ? max_pid_output : PR4_output;
    	Y1_output  = (Y1_output  < min_pid_output) ? min_pid_output : (Y1_output  > max_pid_output) ? max_pid_output : Y1_output;
    	Y2_output  = (Y2_output  < min_pid_output) ? min_pid_output : (Y2_output  > max_pid_output) ? max_pid_output : Y2_output;

    	// Set the Motor Outputs
    	TIM3->CCR1 = PR1_output;    // Left Front
    	TIM3->CCR2 = PR2_output;    // Right Front
    	TIM3->CCR3 = PR3_output;    // Left Rear
    	TIM3->CCR4 = PR4_output;    // Right Rear
    	TIM4->CCR1 = Y1_output;     // CW Yaw
    	TIM4->CCR2 = Y2_output;     // CCW Yaw

        // Update previous errors
        prev_error_pitch = error_pitch;
        prev_error_roll  = error_roll;
        prev_error_yaw   = error_yaw;

    }
