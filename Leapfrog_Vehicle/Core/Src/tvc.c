/* ******************************************************************************
 * @file  : TVC.c
 *
 * @brief : This module reads linear actuator positions, performs angle calculations, 
 *          and includes controls and state handling for navigation.
 * 
 *  Created on: Aug 14, 2024
 *      Author: Viserion
 * 
 *  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "tvc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "processIMU.h"
#include <math.h>

// Define GPIO pins for controlling TVC1 and TVC2
#define TVC1_FWD_PIN GPIO_PIN_15
#define TVC1_REV_PIN GPIO_PIN_14
#define TVC2_FWD_PIN GPIO_PIN_13
#define TVC2_REV_PIN GPIO_PIN_12



// PA4 & PA6 are used to determine the position (based on voltage) of the actuators
// thus they are using the ADC to convert voltage to binary information


// SubsystemState tvcState; // State can be SystemDisabled, SystemManual, or SystemAutomatic

//Variable declarations
TVC_Data latestTVC_Data; 			  // TODO Store current dataset in this packet
uint32_t adcValues[4]; 				  // Stores ADC values for up to 4 channels
uint32_t sum_axis1 = 0;
uint32_t sum_axis2 = 0;
float filter_count = 0;

float tvcManualSetpoint1;             // Currently unused, placeholder for manual control
float tvcManualSetpoint2;
float tvcVal1;
float tvcVal2;
float tvcVal1_angular;
float tvcVal2_angular; 
float set_pos1 = TVC_AXIS1_CENTER;    // Axis_1 (pitch) center is 2350; Defined in header file
float set_pos2 = TVC_AXIS2_CENTER;    // Axis_2 (roll) center is 2300
float roll_imu;                       // replace with pitch and roll data
float pitch_imu;       				  //
float roll_robot;                     // ??
float pitch_robot;                    // ??



//Functions declaration
void Extend(int axis);
void Retract(int axis);
void Halt(int axis);
float AngularToLinear(int axis, float angular);
float LinearToAngular(int axis, float linear);


/* MAIN CONTROL FUNCTION --------------------------------------------------------------- */
void TVC_Controller(void *argument) {
	// TODO Set argument to point to SysManual/Auto for GST control
    // Controller is working like a listener
    // Always on to calculate the difference between actuators values and set_pos (position wanted)

  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Need 100Hz to read IMU
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );  // Wait for the next cycle.
    
    // Low-pass filter to reduced noise on the actuator reading
    if (filter_count < TVC_FILTER_TAPS)
    {
      sum_axis1 += adcValues[0];
      sum_axis2 += adcValues[1];
      filter_count += 1;
    }
    else
    { 
      tvcVal1 = ((float)sum_axis1) / filter_count;
      tvcVal2 = ((float)sum_axis2) / filter_count;
      tvcVal1_angular = LinearToAngular(1, tvcVal1);
      tvcVal2_angular = LinearToAngular(2, tvcVal2);

      // Reset Filter
      filter_count = 0; 
      sum_axis1 = 0;
      sum_axis2 = 0;
    }

    // Check system state and control accordingly
    if (tvcState == SystemAutomatic || tvcState == SystemManual) {

    	// TODO if systemManual, use TVC_Manual

      // Calculate errors for each axis
      float axis1_error = (set_pos1 - tvcVal1);
      float axis2_error = (set_pos2 - tvcVal2);


      // Axis 1 control
      if (fabs(axis1_error) > TVC_DEADBAND) {
        if (axis1_error > 0.0) {
          Extend(1);
        }
        else {
          Retract(1);
        }
      }
      else {
        Hold(1);
      }

      // Axis 2 control
      if (fabs(axis2_error) > TVC_DEADBAND) {
        if (axis2_error > 0.0) {
        	Extend(2);
        }
        else {
        	Retract(2);
        }
      }
      else {
        Hold(2);
      }
    }
    else {
      Hold(1);
      Hold(2);
    }
  }
}


// Function for Manual control, currently a placeholder that sets the gimbal to <0,0>
// Replace with the auto TVC code in main control command
void TVC_Manual(void) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // Need 100Hz or faster to read IMU
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (tvcState == SystemManual) { // Loop only while in manual state
	  // Wait for the next cycle
      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      if (tvcState == SystemManual) {
          // Maintain the home position by setting setpoints to zero
          set_pos1 = AngularToLinear(1, 0);  // Defines the gimbal's target position as <0°, 0°>
          set_pos2 = AngularToLinear(2, 0);

          // Correct positions to match setpoints
          HomePosition(1);
          HomePosition(2);
      }
  }
}


/*
//TVC Auto Task (commented out until ready to add auto TVC)
void StartTVCAutom(void *argument) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // Need 100Hz or faster to read IMU
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    
    IMUMessage message;
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    while (xQueueReceive(tvcimuQueueHandle, &message, 0) == pdTRUE) {
      // Do something with the message
      roll_imu = message.roll_deg;
      pitch_imu = message.pitch_deg;

      //switching from IMU_reference frame to robot reference frame
      //IMU is 45° orientated from the robot axis
      //Set up to move opposite direction from the robot in order to do attitude control 
      roll_robot = 0.5*pitch_imu + 0.5*roll_imu;
      pitch_robot = -0.5*pitch_imu + 0.5*roll_imu;
      if (tvcState == SystemAutomatic)
      {
        if (pitch_robot > TVC_AUTO_DEADBAND_DEG || pitch_robot < -TVC_AUTO_DEADBAND_DEG)
        {
          set_pos1 = AngularToLinear(1, 1*pitch_robot);
        }
        else 
        {
          set_pos1 = AngularToLinear(1,0);
        }
        if (roll_robot > TVC_AUTO_DEADBAND_DEG || roll_robot < -TVC_AUTO_DEADBAND_DEG)
        {
          set_pos2 = AngularToLinear(2, -1*roll_robot);
        }
        else
        {
          set_pos2 = AngularToLinear(2,0);
        }
      }
    }  
  }
}*/


/* GPIO Control Functions -----------------------------------------------------  */

void Extend(int axis) { // TODO Forward and reverse are switched
    if (axis == 1) {
        HAL_GPIO_WritePin(GPIOB, TVC1_FWD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, TVC1_REV_PIN, GPIO_PIN_SET);


    } else if (axis == 2) {
        HAL_GPIO_WritePin(GPIOB, TVC2_FWD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, TVC2_REV_PIN, GPIO_PIN_SET);

    }
}

void Retract(int axis) {
    if (axis == 1) {
        HAL_GPIO_WritePin(GPIOB, TVC1_FWD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, TVC1_REV_PIN, GPIO_PIN_RESET);


    } else if (axis == 2) {
        HAL_GPIO_WritePin(GPIOB, TVC2_FWD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, TVC2_REV_PIN, GPIO_PIN_RESET);

    }
}


void HomePosition(int axis) {
    // This function sets the gimbal to center and holds position.

    // Determine current position and center for the axis called from control function
    float current_pos = (axis == 1) ? adcValues[0] : adcValues[1];
    float center_pos  = (axis == 1) ? TVC_AXIS1_CENTER : TVC_AXIS2_CENTER;

    // Calculate error
    float error = center_pos - current_pos;

    // Correct position based on error
    if (fabs(error) > TVC_DEADBAND) {
        if (error > 0.0) {
            Extend(axis);
        } else {
            Retract(axis);
        }
    } else {
        Hold(axis);
    }
}


void Hold(int axis)
    // This command halts actuator movement

{
	if(axis == 1)
	    {	// Check if the relays are already off
	        if (HAL_GPIO_ReadPin(GPIOB, TVC1_FWD_PIN) == GPIO_PIN_SET &&
	            HAL_GPIO_ReadPin(GPIOB, TVC1_REV_PIN) == GPIO_PIN_SET)
	        { // If both forward and reverse are already off, do nothing

	            return;
	        } // Otherwise, stop the actuator by turning off the relays:
	        HAL_GPIO_WritePin(GPIOB, TVC1_FWD_PIN, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOB, TVC1_REV_PIN, GPIO_PIN_SET);
	    }

	else if(axis == 2)
	    {   // Repeat for axis 2
	        if (HAL_GPIO_ReadPin(GPIOB, TVC2_FWD_PIN) == GPIO_PIN_SET &&
	            HAL_GPIO_ReadPin(GPIOB, TVC2_REV_PIN) == GPIO_PIN_SET)
	        {
	            return;
	        }
	        HAL_GPIO_WritePin(GPIOB, TVC2_FWD_PIN, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOB, TVC2_REV_PIN, GPIO_PIN_SET);
	    }
}


// Angle Conversions
float AngularToLinear(int axis, float angular){
	// Convert Angular values from the gimbal to linear for the actuators
	// Done by measurements
	// Set the limits for each axis when the angle is over 5.5 and -5.5 degree
	// 3100 - 1600 axis 1
	// 3000 - 1600 axis 2
  float linear;
  if (axis == 1)
  {
    if (angular > 5.5){
      linear = 3100;
      return linear;
    }
    else if (angular < - 5.5){
      linear = 1600;
      return linear;
    }
    else{
    linear = (136.96*angular) + TVC_AXIS1_CENTER;
    return linear;
    }
  }

  else if (axis == 2)
  {
    if (angular > 5.5){
      linear = 3000;
      return linear;
    }
    else if (angular < -5.5){
      linear = 1600;
      return linear;
    }
    else{
    linear = (123*angular) + TVC_AXIS2_CENTER;
    return floor(linear);
    }
  }

  else 
  {
    return 0;
  }
}


float LinearToAngular(int axis, float linear)
{
  float angular;
  if (axis == 1)
  {
    angular = (float) (linear - TVC_AXIS1_CENTER) / 136.96;
    return angular;
  }
  else if (axis == 2)
  {
    angular = (float) (linear - TVC_AXIS2_CENTER) / 123.f;
    return angular;
  }
  else 
  {
    return 0;
  }
}


void Test_TVC(int axis){

    // Set to retracted position
    int test_position = 2200;

    // Move the actuator to fixed position
    if (axis == 1) {
        while (fabs((float)test_position - adcValues[0]) > TVC_DEADBAND) {
            if (test_position > adcValues[0]) {
                Extend(1);
            } else {
                Retract(1);
            }
        }

        Hold(100); // Stop movement once the position is reached

    } else if (axis == 2) {
        while (fabs((float)test_position - adcValues[1]) > TVC_DEADBAND) {
            if (test_position > adcValues[1]) {
                Extend(2);
            } else {
                Retract(2);
            }
        }

        Hold(200); // Stop movement once the position is reached
    }
}


