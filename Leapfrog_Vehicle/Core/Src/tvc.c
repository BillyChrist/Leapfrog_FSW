/* ******************************************************************************
 * @file  : TVC.c
 *
 * @brief : This module reads linear actuator positions, performs angle calculations, 
 *          and includes controls and state handling for navigation.
 * 
 *  Created on: Aug 14, 2024
 *      Author: BillyChrist
 * 
 *  ******************************************************************************
 */


// TODO: add calculation to compensate for actuator connection tracing out a circle as it pushes (length of actuation vs gimbal angle will not be linear, since it will have to push longer as angle increases)

/* Includes ------------------------------------------------------------------*/
#include "tvc.h"
#include "main.h"
#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "processIMU.h"
#include "ProcessGimbalIMU.h"
#include "gps.h"
#include "engine.h"
#include <math.h>
#include <string.h>

// Define GPIO pins for controlling TVC1 and TVC2 (axis 1 and 2)
#define TVC1_FWD_PIN GPIO_PIN_15
#define TVC1_REV_PIN GPIO_PIN_14
#define TVC2_FWD_PIN GPIO_PIN_13
#define TVC2_REV_PIN GPIO_PIN_12

// Math constants
#define DEG_TO_RAD (M_PI / 180.0f)


// PA4 & PA6 are used to determine the position (based on potentiometer) of the actuators
// thus they are using the ADC to convert voltage to binary information

// Variable Definitions
#define TVC_DEADBAND 25.0
#define TVC_AXIS1_CENTER 2300
#define TVC_AXIS2_CENTER 2300
#define TVC_FILTER_TAPS 2
#define TVC_AUTO_DEADBAND_DEG 1

// Actuator position limits (ADC values from potentiometers) // TODO add calibration step - limit switch?
// TODO allow calibration to set the max and min positions
#define TVC_AXIS1_MAX_POSITION 3100    // Maximum extension position for axis 1
#define TVC_AXIS1_MIN_POSITION 1600    // Minimum retraction position for axis 1
#define TVC_AXIS2_MAX_POSITION 3000    // Maximum extension position for axis 2
#define TVC_AXIS2_MIN_POSITION 1600    // Minimum retraction position for axis 2

// Angular limits for gimbal control (degrees)
#define TVC_MAX_ANGLE_DEG 5.5          // Maximum angular deflection in degrees
#define TVC_MIN_ANGLE_DEG -5.5         // Minimum angular deflection in degrees

// Conversion factors (ADC units per degree)
#define TVC_AXIS1_DEG_TO_ADC 136.96    // ADC units per degree for axis 1
#define TVC_AXIS2_DEG_TO_ADC 123.0     // ADC units per degree for axis 2

//Variable declarations
TVC_Data latestTVC_Data; 		       // TODO Store current dataset in this packet
uint32_t adcValues[4]; 				   // Stores ADC values for up to 4 channels
uint32_t sum_axis1 = 0;
uint32_t sum_axis2 = 0;
float filter_count = 0;

float tvcManualSetpoint1;             // Currently unused, placeholder for manual control
float tvcManualSetpoint2;
float tvcVal1;
float tvcVal2;
float tvcVal1_angular;
float tvcVal2_angular; 
float set_pos1 = TVC_AXIS1_CENTER;    // Axis_1 (pitch) center
float set_pos2 = TVC_AXIS2_CENTER;    // Axis_2 (roll) center

// Gimbal IMU data - Direct gimbal angle measurement
float gimbal_x_axis_deg;              // X-axis gimbal rotation angle (degrees)
float gimbal_y_axis_deg;              // Y-axis gimbal rotation angle (degrees)
float gimbal_x_axis_velocity_degs;    // X-axis gimbal angular velocity (deg/s)
float gimbal_y_axis_velocity_degs;    // Y-axis gimbal angular velocity (deg/s)
float gimbal_x_axis_accel_g;          // X-axis gimbal acceleration (g)
float gimbal_y_axis_accel_g;          // Y-axis gimbal acceleration (g)
bool gimbal_imu_data_valid = false;   // Gimbal IMU data validity flag

// Fused position feedback (ADC + Gimbal IMU)
float fused_pitch_deg;                // Fused gimbal pitch angle (degrees)
float fused_roll_deg;                 // Fused gimbal roll angle (degrees)
float fused_pitch_velocity_degs;      // Fused gimbal pitch velocity (deg/s)
float fused_roll_velocity_degs;       // Fused gimbal roll velocity (deg/s)

// Attitude compensation variables
float attitude_compensation_roll;     // Roll compensation angle for gimbal
float attitude_compensation_pitch;    // Pitch compensation angle for gimbal
float compensated_pitch;              // Base gimbal angle (0 = level)
float compensated_roll;               // Base gimbal angle (0 = level)

// Command parser variables (now managed by heartbeat.c)
extern CommandParser command_parser;  // Command parser for ground station commands
float current_thrust_n;               // Current thrust output (Newtons)
float jetThrottle_offset;             // Throttle offset for navigation commands

// Compensation algorithm parameters
#define TVC_COMPENSATION_GAIN 1.0f    // Compensation gain (1.0 = full compensation)
#define TVC_DEADBAND_DEG 0.5f         // Deadband - ignore small attitude changes (degrees)
#define TVC_MAX_COMPENSATION_DEG 3.0f // Maximum compensation angle (degrees)

// Navigation mode parameters
#define TVC_MAX_GIMBAL_ANGLE_DEG 5.0f // Maximum gimbal angle for navigation (degrees)
#define TVC_THRUST_NOMINAL_N 100.0f   // Nominal thrust force (Newtons)
#define TVC_VEHICLE_MASS_KG 10.0f     // Vehicle mass (kg)
#define TVC_GRAVITY_MS2 9.81f         // Gravity acceleration (m/s²)

// Thrust-to-angle conversion (based on vehicle dynamics)
#define TVC_THRUST_TO_ANGLE_FACTOR 0.1f // Conversion factor (deg per N of thrust)
float thrust_offset = 0.0f;

// Gimbal IMU sensor integration (placeholder for future implementation)
#define GIMBAL_IMU_ENABLED 0          // Set to 1 when gimbal IMU sensors are installed
#define GIMBAL_IMU_DEADBAND_DEG 0.1f  // Deadband for gimbal angle readings

//Functions declaration
void Extend(int axis);
void Retract(int axis);
void TVC_Controller(void *argument);
void TVC_Manual(void);
void HomePosition(int axis);
void Hold(int axis);
float AngularToLinear(int axis, float angular);
float LinearToAngular(int axis, float linear);

// Attitude compensation functions
void UpdateAttitudeCompensation(void);
void CompensateGimbalAngles(float *target_pitch, float *target_roll);
void ApplyGPSDriftCompensation(float *target_pitch, float *target_roll);
void CalculateDesiredVelocity(float gimbal_pitch, float gimbal_roll, float throttle_percent,
                             float *desired_velocity_north, float *desired_velocity_east);
void UpdateTVCVelocity(void);

// Control algorithm functions
float CalculateThrustOffset(float velocity_ms, float acceleration_ms2, float mass_kg);
void ApplyControlAlgorithm(float *target_pitch, float *target_roll, float *thrust_offset);

// Gimbal IMU integration functions (now implemented)
void updateGimbalIMUData(void);
void GetGimbalIMU_Data(float *x_axis_deg, float *y_axis_deg, 
                       float *x_axis_velocity_degs, float *y_axis_velocity_degs,
                       float *x_axis_accel_g, float *y_axis_accel_g);

// Sensor fusion functions
void FusePositionFeedback(void);
void UpdateFusedPosition(void);
bool isGimbalIMUDataValid(void);

// Command parser functions
void ExecuteCommand(void);
void ClearCommand(void);
float CalculateGimbalAngle(float velocity_ms, float thrust_n);
float CalculateThrottleOffset(float velocity_ms, float distance_m);

// IMU data access functions (vehicle reference frame)
void GetTVC_IMU_Data(float *roll, float *pitch, float *yaw, 
                     float *roll_rate, float *pitch_rate, float *yaw_rate,
                     float *acc_x, float *acc_y, float *acc_z);



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

    // Update TVC velocity from IMU acceleration data
    UpdateTVCVelocity();
    
    // Update fused position feedback (ADC + Gimbal IMU)
    UpdateFusedPosition();

    // Check system state and control accordingly
    if (tvcState == SystemTVC_Automatic) {
      // Check for new commands from ground station
      if (command_parser.new_command) {
        ExecuteCommand();
      }
      
      // If executing a command, continue execution
      if (command_parser.executing) {
        // Command is being executed, gimbal angles already set
        // Just need to check if command duration has expired
        uint32_t current_time = HAL_GetTick();
        float elapsed_time = (current_time - command_parser.start_time_ms) / 1000.0f;
        
        if (elapsed_time >= command_parser.duration_s) {
          // Command completed, clear and return to hover
          ClearCommand();
          // Set status message for ground station
          extern void SetStatusMessage(const char* message);
          SetStatusMessage("Navigation Complete");
        }
      } else {
        // No command executing, maintain level position with attitude compensation
        compensated_pitch = 0.0f;  // Base gimbal angle (0 = level)
        compensated_roll = 0.0f;   // Base gimbal angle (0 = level)
        UpdateAttitudeCompensation();
        CompensateGimbalAngles(&compensated_pitch, &compensated_roll);
        
        // Apply GPS drift compensation if GPS data is valid
        ApplyGPSDriftCompensation(&compensated_pitch, &compensated_roll);
      }
      
      // Convert compensated angles to linear positions
      set_pos1 = AngularToLinear(1, compensated_pitch);
      set_pos2 = AngularToLinear(2, compensated_roll);
    } else if (tvcState == SystemTVC_Disable) {
      // TVC disabled - hold current position on both actuators
      set_pos1 = tvcVal1;  // Hold current position
      set_pos2 = tvcVal2;  // Hold current position
    }
    
    // Calculate errors for each axis using fused position feedback
    if (tvcState == SystemTVC_Automatic || tvcState == SystemTVC_Disable) {
      // Convert fused angles back to linear positions for error calculation
      float current_pos1 = AngularToLinear(1, fused_pitch_deg);
      float current_pos2 = AngularToLinear(2, fused_roll_deg);
      
      float axis1_error = (set_pos1 - current_pos1);
      float axis2_error = (set_pos2 - current_pos2);


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







/* GPIO Control Functions -----------------------------------------------------  */
void Extend(int axis) { // TODO Forward and reverse are switched
    if (axis == 1) {
        // Axis 1 (pitch): De-energize forward, energize reverse
        HAL_GPIO_WritePin(GPIOB, TVC1_FWD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, TVC1_REV_PIN, GPIO_PIN_SET);
    } else if (axis == 2) {
        // Axis 2 (roll): De-energize forward, energize reverse
        HAL_GPIO_WritePin(GPIOB, TVC2_FWD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, TVC2_REV_PIN, GPIO_PIN_SET);
    }
}

void Retract(int axis) {
    if (axis == 1) {
        // Axis 1 (pitch): Energize forward, de-energize reverse
        HAL_GPIO_WritePin(GPIOB, TVC1_FWD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, TVC1_REV_PIN, GPIO_PIN_RESET);
    } else if (axis == 2) {
        // Axis 2 (roll): Energize forward, de-energize reverse
        HAL_GPIO_WritePin(GPIOB, TVC2_FWD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, TVC2_REV_PIN, GPIO_PIN_RESET);
    }
}

void Hold(int axis){
    // This command halts actuator movement
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


/* Helper Functions --------------------------------------------------------------- */


/**
 * @brief Execute parsed command
 * 
 * This function executes the parsed command by calculating gimbal angles
 * and throttle offset, then starts command execution.
 */
void ExecuteCommand(void) {
    // Calculate gimbal angle and throttle offset
    command_parser.gimbal_angle_deg = CalculateGimbalAngle(command_parser.velocity_ms, current_thrust_n);
    command_parser.jetThrottle_offset = CalculateThrottleOffset(command_parser.velocity_ms, command_parser.distance_m);
    command_parser.duration_s = command_parser.distance_m / command_parser.velocity_ms;
    
    // Set gimbal angles based on direction
    if (strcmp(command_parser.direction, "forward") == 0) {
        compensated_pitch = command_parser.gimbal_angle_deg;
        compensated_roll = 0.0f;
    } else if (strcmp(command_parser.direction, "backward") == 0) {
        compensated_pitch = -command_parser.gimbal_angle_deg;
        compensated_roll = 0.0f;
    } else if (strcmp(command_parser.direction, "left") == 0) {
        compensated_pitch = 0.0f;
        compensated_roll = command_parser.gimbal_angle_deg;
    } else if (strcmp(command_parser.direction, "right") == 0) {
        compensated_pitch = 0.0f;
        compensated_roll = -command_parser.gimbal_angle_deg;
    }
    
    // Apply attitude compensation
    UpdateAttitudeCompensation();
    CompensateGimbalAngles(&compensated_pitch, &compensated_roll);
    
    // Start command execution
    command_parser.executing = true;
    command_parser.new_command = false;
    command_parser.start_time_ms = HAL_GetTick();
    
    // Set throttle offset (dummy for now)
    jetThrottle_offset = command_parser.jetThrottle_offset;
    
    // Set status message for ground station
    extern void SetStatusMessage(const char* message);
    SetStatusMessage("Command Executing");
}


/**
 * @brief Clear command and return to hover
 */
void ClearCommand(void) {
    command_parser.executing = false;
    command_parser.new_command = false;
    jetThrottle_offset = 0.0f;  // Reset throttle offset
}



void TVC_Calibration(void) {
    // TODO: Implement TVC calibration logic
    // This could include:
    // - find min/max positions for actuators
    // - use IMU angles to find center position
    // - calibrate IMUs to find offsets
}


    
void Hover(void) {
    // This function should call the compensation functions to get the vehicle to hover at the current position.
    // attitude compensation, and IMU/GPS drift compensation.
    // This function should be also called in the main control loop when given navigation commands.
    // This should be the default mode while waiting for a navigation command... after nav is complete > go to hover.
}


void HomePosition(int axis) {
    // This function sets the gimbal to center and holds position.
    // get the updated max/min positions
    // Determine current position and center for the axis (thrust vector pointing down) called from control function
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



/**
 * @brief Convert angular gimbal position to linear actuator ADC value
 * @param axis: TVC axis (1 = pitch, 2 = roll)
 * @param angular: Desired angle in degrees (-5.5° to +5.5°)
 * @return Linear actuator position as ADC value (1600-3100 for axis 1, 1600-3000 for axis 2)
 * 
 * This function converts the desired gimbal angle to the corresponding linear actuator
 * position. The conversion is based on physical measurements of the actuator range.
 * Angles beyond ±5.5° are clamped to the actuator limits.
 */
float AngularToLinear(int axis, float angular){
  float linear;
  
  if (axis == 1)  // Pitch axis
  {
    // Clamp angle to physical limits and convert to ADC value
    if (angular > TVC_MAX_ANGLE_DEG){
      linear = TVC_AXIS1_MAX_POSITION;  // Maximum extension
      return linear;
    }
    else if (angular < TVC_MIN_ANGLE_DEG){
      linear = TVC_AXIS1_MIN_POSITION;  // Maximum retraction
      return linear;
    }
    else{
      // Linear conversion: angle * conversion_factor + center_position
      linear = (TVC_AXIS1_DEG_TO_ADC * angular) + TVC_AXIS1_CENTER;
    return linear;
    }
  }

  else if (axis == 2)  // Roll axis
  {
    // Clamp angle to physical limits and convert to ADC value
    if (angular > TVC_MAX_ANGLE_DEG){
      linear = TVC_AXIS2_MAX_POSITION;  // Maximum extension
      return linear;
    }
    else if (angular < TVC_MIN_ANGLE_DEG){
      linear = TVC_AXIS2_MIN_POSITION;  // Maximum retraction
      return linear;
    }
    else{
      // Linear conversion: angle * conversion_factor + center_position
      linear = (TVC_AXIS2_DEG_TO_ADC * angular) + TVC_AXIS2_CENTER;
      return floor(linear);  // Round down to integer ADC value
    }
  }

  else 
  {
    return 0;  // Invalid axis
  }
}

/**
 * @brief Convert linear actuator ADC value to angular gimbal position
 * @param axis: TVC axis (1 = pitch, 2 = roll)
 * @param linear: Actuator position as ADC value from potentiometer
 * @return Angular position in degrees
 * 
 * This function converts the current linear actuator position (from ADC reading)
 * back to the corresponding gimbal angle. This is used for feedback and monitoring.
 * The conversion is the inverse of AngularToLinear().
 */
float LinearToAngular(int axis, float linear)
{
  float angular;
  
  if (axis == 1)  // Pitch axis
  {
    // Inverse conversion: (ADC_value - center) / conversion_factor
    angular = (float) (linear - TVC_AXIS1_CENTER) / TVC_AXIS1_DEG_TO_ADC;
    return angular;
  }
  else if (axis == 2)  // Roll axis
  {
    // Inverse conversion: (ADC_value - center) / conversion_factor
    angular = (float) (linear - TVC_AXIS2_CENTER) / TVC_AXIS2_DEG_TO_ADC;
    return angular;
  }
  else 
  {
    return 0;  // Invalid axis
  }
}


/**
 * @brief Get comprehensive TVC IMU data (angles, velocities, accelerations)
 * @param roll: Pointer to store roll angle (degrees) - can be NULL
 * @param pitch: Pointer to store pitch angle (degrees) - can be NULL
 * @param yaw: Pointer to store yaw angle (degrees) - can be NULL
 * @param roll_rate: Pointer to store roll rate (deg/s) - can be NULL
 * @param pitch_rate: Pointer to store pitch rate (deg/s) - can be NULL
 * @param yaw_rate: Pointer to store yaw rate (deg/s) - can be NULL
 * @param acc_x: Pointer to store X acceleration (g) - can be NULL
 * @param acc_y: Pointer to store Y acceleration (g) - can be NULL
 * @param acc_z: Pointer to store Z acceleration (g) - can be NULL
 * 
 * Pass NULL for any parameters you don't need to improve performance.
 */
void GetTVC_IMU_Data(float *roll, float *pitch, float *yaw, 
    float *roll_rate, float *pitch_rate, float *yaw_rate,
    float *acc_x, float *acc_y, float *acc_z) {
// Attitude data
if (roll != NULL) *roll = tvc_roll_deg;
if (pitch != NULL) *pitch = tvc_pitch_deg;
if (yaw != NULL) *yaw = tvc_yaw_deg;

// Angular velocity data
if (roll_rate != NULL) *roll_rate = tvc_angvel_x_degs;
if (pitch_rate != NULL) *pitch_rate = tvc_angvel_y_degs;
if (yaw_rate != NULL) *yaw_rate = tvc_angvel_z_degs;

// Acceleration data
if (acc_x != NULL) *acc_x = tvc_acc_x_g;
if (acc_y != NULL) *acc_y = tvc_acc_y_g;
if (acc_z != NULL) *acc_z = tvc_acc_z_g;
}


/**
 * @brief Update attitude compensation values from TVC IMU data
 * 
 * This function calculates the required gimbal compensation to maintain
 * pointing direction despite vehicle attitude changes. The compensation
 * is based on how far the vehicle is from perfectly level (0° pitch, 0° roll).
 * 
 * Algorithm:
 * 1. Calculate deviation from level (current attitude - 0°)
 * 2. Apply deadband to ignore small changes
 * 3. Apply compensation gain
 * 4. Clamp to maximum compensation limits
 * 
 * Should be called at the same frequency as the TVC controller (100Hz)
 * to ensure smooth attitude compensation.
 */
void UpdateAttitudeCompensation(void) {
    // Calculate deviation from perfectly level (0° pitch, 0° roll)
    float roll_deviation  = tvc_roll_deg  - 0.0f;    // How far from level roll TODO Why - 0? why redefine a new term? This TVC_roll_deg is essentially already the deviation.
    float pitch_deviation = tvc_pitch_deg - 0.0f;    // How far from level pitch
    
    // Apply deadband - ignore small attitude changes
    if (fabs(roll_deviation) < TVC_DEADBAND_DEG) {
        roll_deviation = 0.0f;
    }
    if (fabs(pitch_deviation) < TVC_DEADBAND_DEG) {
        pitch_deviation = 0.0f;
    }
    
    // Calculate compensation (gimbal moves opposite to vehicle deviation)
    attitude_compensation_roll  = -roll_deviation * TVC_COMPENSATION_GAIN;
    attitude_compensation_pitch = -pitch_deviation * TVC_COMPENSATION_GAIN;
    
    // Clamp compensation to maximum limits
    if (attitude_compensation_roll > TVC_MAX_COMPENSATION_DEG) {
        attitude_compensation_roll = TVC_MAX_COMPENSATION_DEG;
    } else if (attitude_compensation_roll < -TVC_MAX_COMPENSATION_DEG) {
        attitude_compensation_roll = -TVC_MAX_COMPENSATION_DEG;
    }
    
    if (attitude_compensation_pitch > TVC_MAX_COMPENSATION_DEG) {
        attitude_compensation_pitch = TVC_MAX_COMPENSATION_DEG;
    } else if (attitude_compensation_pitch < -TVC_MAX_COMPENSATION_DEG) {
        attitude_compensation_pitch = -TVC_MAX_COMPENSATION_DEG;
    }
}


/**
 * @brief Compensate gimbal angles for vehicle attitude changes
 * @param target_pitch: Pointer to target pitch angle (modified in place)
 * @param target_roll: Pointer to target roll angle (modified in place)
 * 
 * This function modifies the target gimbal angles to compensate for
 * vehicle attitude changes, ensuring the gimbal maintains its intended
 * pointing direction despite vehicle tilting.
 * 
 * Example: If vehicle pitches +2°, gimbal compensates -2° to maintain pointing
 */
void CompensateGimbalAngles(float *target_pitch, float *target_roll) {
    if (target_pitch != NULL && target_roll != NULL) {
        // Apply attitude compensation to maintain pointing direction
        // Use fused position data for more accurate compensation
        *target_pitch = fused_pitch_deg + attitude_compensation_pitch;
        *target_roll = fused_roll_deg + attitude_compensation_roll;
        
        // Clamp to gimbal physical limits
        if (*target_pitch > TVC_MAX_ANGLE_DEG) {
            *target_pitch = TVC_MAX_ANGLE_DEG;
        } else if (*target_pitch < TVC_MIN_ANGLE_DEG) {
            *target_pitch = TVC_MIN_ANGLE_DEG;
        }
        
        if (*target_roll > TVC_MAX_ANGLE_DEG) {
            *target_roll = TVC_MAX_ANGLE_DEG;
        } else if (*target_roll < TVC_MIN_ANGLE_DEG) {
            *target_roll = TVC_MIN_ANGLE_DEG;
        }
    }
}


/**
 * @brief Calculate required gimbal angle for desired velocity
 * @param velocity_ms: Desired velocity in m/s
 * @param thrust_n: Current thrust output in Newtons
 * @return Required gimbal angle in degrees
 */
float CalculateGimbalAngle(float velocity_ms, float thrust_n) {
    // Calculate required acceleration
    float required_acceleration = velocity_ms; // m/s²
    
    // Calculate required horizontal force
    float required_horizontal_force = TVC_VEHICLE_MASS_KG * required_acceleration;
    
    // Calculate gimbal angle
    float gimbal_angle_rad = atan(required_horizontal_force / thrust_n);
    float gimbal_angle_deg = gimbal_angle_rad * 180.0f / 3.14159f;
    
    // Clamp to maximum gimbal angle
    if (gimbal_angle_deg > TVC_MAX_GIMBAL_ANGLE_DEG) {
        gimbal_angle_deg = TVC_MAX_GIMBAL_ANGLE_DEG;
    } else if (gimbal_angle_deg < -TVC_MAX_GIMBAL_ANGLE_DEG) {
        gimbal_angle_deg = -TVC_MAX_GIMBAL_ANGLE_DEG;
    }
    
    return gimbal_angle_deg;
}

/**
 * @brief Calculate throttle offset for navigation command
 * @param velocity_ms: Desired velocity in m/s
 * @param distance_m: Distance to move in meters
 * @return Throttle offset percentage
 */
float CalculateThrottleOffset(float velocity_ms, float distance_m) {
    // Dummy calculation for now
    // In reality, this would calculate required thrust increase
    // based on velocity and distance requirements
    // TODO Check hardcoded values, is this a tunable parameter? 
    return velocity_ms * 10.0f;  // Simple linear relationship
}

/**
 * @brief Apply drift compensation using sensor fusion
 * @param target_pitch: Target pitch angle (modified in place)
 * @param target_roll: Target roll angle (modified in place)
 * 
 * This function implements sensor fusion between IMU and GPS data to detect
 * and compensate for drift using physics-based calculations.
 */
// TODO Don't implicitly declare variables within a function. all floats should be declared at the top of the script if possible.
// TODO also there should be no hard-coded values like: float imu_weight = 0.7f;  // Higher weight for IMU (more responsive)
float gps_weight = 0.3f;  // Lower weight for GPS (more stable)
void ApplyGPSDriftCompensation(float *target_pitch, float *target_roll) {
    // Get current IMU velocity data (high frequency, short-term accurate)
    float imu_velocity_north_ms = tvc_velocity_north_ms;
    float imu_velocity_east_ms = tvc_velocity_east_ms;
    
    // Get GPS velocity data (low frequency, long-term accurate)
    float gps_velocity_north_ms, gps_velocity_east_ms;
    gpsGetVelocity(&gps_velocity_north_ms, &gps_velocity_east_ms);
    
    // Get GPS position error
    float north_error_m, east_error_m;
    gpsGetPositionError(&north_error_m, &east_error_m);
    
    // Sensor fusion: Prioritize IMU for short-term, GPS for long-term
    float fused_velocity_north_ms, fused_velocity_east_ms;
    float imu_weight = 0.7f;  // Higher weight for IMU (more responsive)
    float gps_weight = 0.3f;  // Lower weight for GPS (more stable)
    
    // Check if GPS data is valid and recent
    extern GPS_Data latestGPSdata;
    bool gps_valid = gpsIsDataValid(&latestGPSdata);
    
    if (gps_valid) {
        // Sensor fusion: Weighted average of IMU and GPS velocities
        fused_velocity_north_ms = imu_weight * imu_velocity_north_ms + gps_weight * gps_velocity_north_ms;
        fused_velocity_east_ms = imu_weight * imu_velocity_east_ms + gps_weight * gps_velocity_east_ms;
    } else {
        // Fallback to IMU only if GPS is invalid
        fused_velocity_north_ms = imu_velocity_north_ms;
        fused_velocity_east_ms = imu_velocity_east_ms;
    }
    
    // Calculate desired velocity based on current gimbal angle and throttle
    float current_throttle_percent = 50.0f; // TODO: Get from engine system
    float desired_velocity_north_ms, desired_velocity_east_ms;
    CalculateDesiredVelocity(*target_pitch, *target_roll, current_throttle_percent, 
                            &desired_velocity_north_ms, &desired_velocity_east_ms);
    
    // Calculate velocity error (drift)
    float velocity_error_north_ms = fused_velocity_north_ms - desired_velocity_north_ms;
    float velocity_error_east_ms = fused_velocity_east_ms - desired_velocity_east_ms;
    
    // Calculate position error contribution
    float position_error_north_ms = north_error_m * 0.1f; // Convert position error to velocity
    float position_error_east_ms = east_error_m * 0.1f;
    
    // Total drift compensation
    float total_drift_north_ms = velocity_error_north_ms + position_error_north_ms;
    float total_drift_east_ms = velocity_error_east_ms + position_error_east_ms;
    
    // Physics-based compensation: Calculate required gimbal angle change
    float compensation_gain = 0.8f; // Adjust based on system response
    float max_compensation_deg = 3.0f; // Maximum compensation angle
    
    // Calculate compensation angles based on drift velocity
    float pitch_compensation = -total_drift_north_ms * compensation_gain;
    float roll_compensation = -total_drift_east_ms * compensation_gain;
    
    // Apply limits
    if (pitch_compensation > max_compensation_deg) {
        pitch_compensation = max_compensation_deg;
    } else if (pitch_compensation < -max_compensation_deg) {
        pitch_compensation = -max_compensation_deg;
    }
    
    if (roll_compensation > max_compensation_deg) {
        roll_compensation = max_compensation_deg;
    } else if (roll_compensation < -max_compensation_deg) {
        roll_compensation = -max_compensation_deg;
    }
    
    // Apply compensation to target angles
    *target_pitch += pitch_compensation;
    *target_roll += roll_compensation;
}

/**
 * @brief Calculate desired velocity based on gimbal angle and throttle
 * @param gimbal_pitch: Current gimbal pitch angle in degrees
 * @param gimbal_roll: Current gimbal roll angle in degrees
 * @param throttle_percent: Engine throttle percentage (0-100)
 * @param desired_velocity_north: Desired north velocity (output)
 * @param desired_velocity_east: Desired east velocity (output)
 * 
 * This function uses physics to calculate what velocity should result from
 * the current gimbal angle and throttle setting.
 */
void CalculateDesiredVelocity(float gimbal_pitch, float gimbal_roll, float throttle_percent,
                             float *desired_velocity_north, float *desired_velocity_east) {
    // Convert throttle percentage to thrust force (simplified model)
    float max_thrust_n = 1000.0f; // Maximum thrust in Newtons (adjust based on engine)
    float thrust_n = (throttle_percent / 100.0f) * max_thrust_n;
    
    // TODO All measurements and constants or configurable parameters should be listed at the top of the script, and not hidden within functions. (i.e. vehicle mass, velocity gain,
    // Vehicle mass (adjust based on actual vehicle)
    float vehicle_mass_kg = 50.0f; // kg
    
    // Calculate acceleration from thrust
    float acceleration_ms2 = thrust_n / vehicle_mass_kg;
    
    // Convert gimbal angles to acceleration components
    // Pitch angle affects north-south acceleration
    // Roll angle affects east-west acceleration
    float pitch_rad = gimbal_pitch * DEG_TO_RAD;
    float roll_rad = gimbal_roll * DEG_TO_RAD;
    
    // Calculate desired acceleration components
    float desired_accel_north = acceleration_ms2 * sinf(pitch_rad);
    float desired_accel_east = acceleration_ms2 * sinf(roll_rad);
    
    // Convert acceleration to velocity (simplified: assume steady state)
    // In reality, this would integrate acceleration over time
    float velocity_gain = 1.0f; // Adjust based on system dynamics
    
    *desired_velocity_north = desired_accel_north * velocity_gain;
    *desired_velocity_east = desired_accel_east * velocity_gain;
}

/**
 * @brief Update TVC velocity from IMU acceleration data
 * 
 * This function calculates velocity by integrating acceleration data from the IMU.
 * It should be called regularly in the TVC control loop.
 */
void UpdateTVCVelocity(void) {
    // Static variables to maintain state between calls
	// TODO All these variable terms should be declared at the top of the script, not within a function, even gravity can be defined g = 9.81...
    static float prev_velocity_north = 0.0f;
    static float prev_velocity_east = 0.0f;
    static float prev_velocity_up = 0.0f;
    static uint32_t prev_time_ms = 0;
    
    uint32_t current_time_ms = HAL_GetTick();
    float dt = (current_time_ms - prev_time_ms) / 1000.0f; // Convert to seconds
    
    if (prev_time_ms != 0 && dt > 0.001f && dt < 1.0f) { // Valid time difference
        // Convert acceleration from g to m/s²
        float acc_north_ms2 = tvc_acc_y_g * 9.81f; // Y acceleration = North (assuming vehicle orientation)
        float acc_east_ms2 = tvc_acc_x_g * 9.81f;  // X acceleration = East
        float acc_up_ms2 = tvc_acc_z_g * 9.81f;    // Z acceleration = Up
        
        // Simple integration: v = v0 + a*dt
        tvc_velocity_north_ms = prev_velocity_north + acc_north_ms2 * dt;
        tvc_velocity_east_ms = prev_velocity_east + acc_east_ms2 * dt;
        tvc_velocity_up_ms = prev_velocity_up + acc_up_ms2 * dt;
        
        // Apply simple damping to prevent velocity from growing indefinitely
        float damping_factor = 0.95f;
        tvc_velocity_north_ms *= damping_factor;
        tvc_velocity_east_ms *= damping_factor;
        tvc_velocity_up_ms *= damping_factor;
    }
    
    // Update previous values
    prev_velocity_north = tvc_velocity_north_ms;
    prev_velocity_east = tvc_velocity_east_ms;
    prev_velocity_up = tvc_velocity_up_ms;
    prev_time_ms = current_time_ms;
}


/**
 * @brief Calculate thrust offset for velocity control (placeholder for physical measurements)
 * @param velocity_ms: Current velocity in m/s
 * @param acceleration_ms2: Desired acceleration in m/s²
 * @param mass_kg: Vehicle mass in kg
 * @return Required thrust offset in Newtons
 * 
 * TODO: This function needs to be calibrated based on physical system measurements.
 * The current implementation is a simplified placeholder.
 */
float CalculateThrustOffset(float velocity_ms, float acceleration_ms2, float mass_kg) {
    // TODO import JetCatPRO documentation... 
    // PLACEHOLDER: Insert thrust control algorithm here
    // This will depend on physical measurements of the system including:
    // - Engine throttle response
    // - Thrust vs throttle relationship
    // - Vehicle drag characteristics
    // - Altitude and atmospheric conditions
    
    // Simplified placeholder calculation
    // In reality, this would be based on:
    // 1. Engine performance curves
    // 2. Throttle-to-thrust mapping
    // 3. Drag force calculations
    // 4. Altitude compensation
    
    
    // Basic physics: F = ma
    // But we need to account for:
    // - Current thrust level
    // - Drag forces
    // - Engine response time
    // - Throttle limits
    
    // PLACEHOLDER: Simple proportional control
    // This should be replaced with actual system measurements
    thrust_offset = acceleration_ms2 * mass_kg;
    
    // Apply velocity-dependent drag compensation
    // This is a simplified model - real system would need:
    // - Drag coefficient measurements
    // - Air density calculations
    // - Velocity-squared drag relationship
    float drag_compensation = velocity_ms * velocity_ms * 0.1f; // Placeholder drag factor
    thrust_offset += drag_compensation;
    
    return thrust_offset;
}

/**
 * @brief Apply main control algorithm (placeholder for physical system integration)
 * @param target_pitch: Target pitch angle (modified in place)
 * @param target_roll: Target roll angle (modified in place)
 * @param thrust_offset: Thrust offset in Newtons (output)
 * 
 * TODO: This function needs to be implemented based on physical system measurements.
 * The current implementation is a placeholder framework.
 */
void ApplyControlAlgorithm(float *target_pitch, float *target_roll, float *thrust_offset) {
    // PLACEHOLDER: Insert main control algorithm here
    // This is where the core TVC control logic would be implemented based on:
    // - Vehicle dynamics modeling
    // - Thrust vector control theory
    // - Real-time system measurements
    // - Control system design (PID, LQR, etc.)
    
    // Current implementation is a placeholder that:
    // 1. Gets current system state
    // 2. Calculates desired outputs
    // 3. Applies control logic
    // 4. Outputs gimbal angles and thrust offset
    
    // Get current system state
    float current_velocity_north = tvc_velocity_north_ms;
    float current_velocity_east = tvc_velocity_east_ms;
    
    // Get current thrust from engine system (throttle percentage 0-100)
    extern float throttle;
    float current_thrust = (throttle / 100.0f) * TVC_THRUST_NOMINAL_N;
    
    // Calculate desired velocity (from navigation commands or hover)
    float desired_velocity_north = 0.0f; // TODO: Get from command system
    float desired_velocity_east = 0.0f;  // TODO: Get from command system
    
    // Calculate velocity errors
    float velocity_error_north = desired_velocity_north - current_velocity_north;
    float velocity_error_east = desired_velocity_east - current_velocity_east;
    
    // PLACEHOLDER: Control algorithm implementation
    // This should be replaced with actual control system design
    // Options include:
    // - PID control
    // - LQR (Linear Quadratic Regulator)
    // - Model Predictive Control (MPC)
    // - Adaptive control
    
    // Simple proportional control (placeholder)
    float control_gain = 0.5f; // TODO: Tune based on system response
    
    *target_pitch = velocity_error_north * control_gain;
    *target_roll = velocity_error_east * control_gain;
    
    // Calculate thrust offset for velocity control
    float desired_acceleration = sqrtf(velocity_error_north * velocity_error_north + 
                                     velocity_error_east * velocity_error_east);
    
    // Use current thrust as baseline for thrust offset calculation
    *thrust_offset = CalculateThrustOffset(current_velocity_north, desired_acceleration, TVC_VEHICLE_MASS_KG);
    
    // Scale thrust offset based on current thrust level
    if (current_thrust > 0.0f) {
        *thrust_offset = (*thrust_offset * current_thrust) / TVC_THRUST_NOMINAL_N;
    }
    
    // Apply limits
    if (*target_pitch > TVC_MAX_ANGLE_DEG) {
        *target_pitch = TVC_MAX_ANGLE_DEG;
    } else if (*target_pitch < -TVC_MAX_ANGLE_DEG) {
        *target_pitch = -TVC_MAX_ANGLE_DEG;
    }
    
    if (*target_roll > TVC_MAX_ANGLE_DEG) {
        *target_roll = TVC_MAX_ANGLE_DEG;
    } else if (*target_roll < -TVC_MAX_ANGLE_DEG) {
        *target_roll = -TVC_MAX_ANGLE_DEG;
    }
}

/**
 * @brief Update gimbal IMU data (now implemented with ProcessGimbalIMU)
 * 
 * This function now uses the dedicated ProcessGimbalIMU module for actual
 * gimbal angle measurement from dedicated IMU sensors on the gimbal inner stage.
 */
void UpdateGimbalIMUData(void) {
    if (GIMBAL_IMU_ENABLED) {
        // Use dedicated gimbal IMU processing
        updateGimbalIMUData();
        
        // Get actual gimbal angles from IMU sensors
        GetGimbalIMU_Data(&gimbal_x_axis_deg, &gimbal_y_axis_deg,
                          &gimbal_x_axis_velocity_degs, &gimbal_y_axis_velocity_degs,
                          &gimbal_x_axis_accel_g, &gimbal_y_axis_accel_g);
        
        // Check if data is valid
        gimbal_imu_data_valid = isGimbalIMUDataValid();
    } else {
        // Fallback: Use actuator position as gimbal angle estimate
        gimbal_x_axis_deg = tvcVal1_angular;  // X-axis gimbal = pitch actuator
        gimbal_y_axis_deg = tvcVal2_angular;  // Y-axis gimbal = roll actuator
        gimbal_x_axis_velocity_degs = 0.0f;   // TODO: Calculate from angle changes
        gimbal_y_axis_velocity_degs = 0.0f;   // TODO: Calculate from angle changes
        gimbal_imu_data_valid = true;
    }
}

/**
 * @brief Fuse position feedback from ADC and gimbal IMU sensors
 * 
 * This function implements sensor fusion between:
 * - ADC position feedback (from potentiometers on linear actuators)
 * - Gimbal IMU data (direct angle measurement from IMUs on gimbal stages)
 * 
 * The fusion provides redundant closed-loop control for improved accuracy
 * and fault detection capabilities.
 */
void FusePositionFeedback(void) {
    // Define fusion weights (adjustable based on sensor reliability)
    float adc_weight = 0.6f;        // Weight for ADC position feedback
    float imu_weight = 0.4f;        // Weight for gimbal IMU data
    
    // Check if gimbal IMU data is valid
    if (gimbal_imu_data_valid && isGimbalIMUDataValid()) {
        // Both sensors available - use weighted fusion
        fused_pitch_deg = (adc_weight * tvcVal1_angular) + (imu_weight * gimbal_x_axis_deg);
        fused_roll_deg = (adc_weight * tvcVal2_angular) + (imu_weight * gimbal_y_axis_deg);
        
        // Fuse velocities (IMU provides direct measurement, ADC requires differentiation)
        fused_pitch_velocity_degs = gimbal_x_axis_velocity_degs;  // IMU is more accurate for velocity
        fused_roll_velocity_degs = gimbal_y_axis_velocity_degs;
        
        // TODO: Add fault detection - compare ADC and IMU readings for consistency
        // If difference exceeds threshold, flag sensor fault and adjust weights
        
    } else {
        // Fallback: Use ADC position feedback only
        fused_pitch_deg = tvcVal1_angular;
        fused_roll_deg = tvcVal2_angular;
        fused_pitch_velocity_degs = 0.0f;  // TODO: Calculate from angle changes
        fused_roll_velocity_degs = 0.0f;   // TODO: Calculate from angle changes
    }
}

/**
 * @brief Update fused position data
 * 
 * This function should be called regularly in the control loop to maintain
 * up-to-date fused position feedback for the TVC control algorithm.
 */
void UpdateFusedPosition(void) {
    // Update gimbal IMU data first
    UpdateGimbalIMUData();
    
    // Perform sensor fusion
    FusePositionFeedback();
}

/**
 * @brief Get comprehensive gimbal IMU data (angles, velocities, accelerations)
 * @param x_axis_deg: X-axis gimbal rotation angle (degrees) - can be NULL
 * @param y_axis_deg: Y-axis gimbal rotation angle (degrees) - can be NULL
 * @param x_axis_velocity_degs: X-axis gimbal angular velocity (deg/s) - can be NULL
 * @param y_axis_velocity_degs: Y-axis gimbal angular velocity (deg/s) - can be NULL
 * @param x_axis_accel_g: X-axis gimbal acceleration (g) - can be NULL
 * @param y_axis_accel_g: Y-axis gimbal acceleration (g) - can be NULL
 * Pass NULL for any parameters you don't need to improve performance.
 */
void GetGimbalIMU_Data(float *x_axis_deg, float *y_axis_deg, 
                       float *x_axis_velocity_degs, float *y_axis_velocity_degs,
                       float *x_axis_accel_g, float *y_axis_accel_g) {
    // Gimbal angle data
    if (x_axis_deg != NULL) *x_axis_deg = gimbal_x_axis_deg;
    if (y_axis_deg != NULL) *y_axis_deg = gimbal_y_axis_deg;
    
    // Gimbal velocity data
    if (x_axis_velocity_degs != NULL) *x_axis_velocity_degs = gimbal_x_axis_velocity_degs;
    if (y_axis_velocity_degs != NULL) *y_axis_velocity_degs = gimbal_y_axis_velocity_degs;
    
    // Gimbal acceleration data
    if (x_axis_accel_g != NULL) *x_axis_accel_g = gimbal_x_axis_accel_g;
    if (y_axis_accel_g != NULL) *y_axis_accel_g = gimbal_y_axis_accel_g;
}


void TVC_Manual(void) {
    // Manual TVC control mode
    // This function handles manual gimbal control when TVC is in manual mode
    // It should read manual inputs and control the gimbal accordingly
    
    // TODO: Implement manual control logic
    // This could include:
    // - Reading manual control inputs (joystick, etc.)
    // - Converting inputs to gimbal angles
    // - Applying manual gimbal control
    // - Safety checks and limits
    
    // Placeholder implementation
    // In manual mode, the gimbal should respond to manual inputs
    // For now, this is a placeholder that does nothing
}



/* Test Scripts -------------------------------------------------------------- */
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

