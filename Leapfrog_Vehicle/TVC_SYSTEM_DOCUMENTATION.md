# TVC (Thrust Vector Control) System Documentation

## Overview
The TVC system provides precise thrust vector control for the Leapfrog vehicle through gimbal angle adjustments. It integrates IMU data, GPS positioning, and command-driven navigation to maintain vehicle stability and execute precise maneuvers.

## System Architecture

### Core Components
- **TVC Controller** (`tvc2.c`): Main control loop running at 100Hz
- **IMU Integration** (`ProcessIMU.c`): Provides acceleration and attitude data
- **GPS Integration** (`gps.c`): Provides position, velocity, and drift compensation
- **Heartbeat Communication** (`heartbeat.c`): Command reception and status reporting
- **Actuator Control**: Direct GPIO control of linear actuators

### Hardware Interface
- **Actuators**: 2-axis linear actuators (Pitch/Roll)
- **Position Feedback**: Potentiometers via ADC (PA4, PA6)
- **Control Pins**: GPIO PB12-PB15 for actuator control
- **IMU Data**: 3-axis accelerometer and gyroscope data
- **GPS Module**: UART-based position and velocity data

## Control Modes

### 1. System States
- **SystemTVC_Enable**: Active control with attitude compensation and drift correction
- **SystemTVC_Disable**: Hold current position, no active control

### 2. Operational Modes
- **Hover Mode**: Maintain level position with attitude compensation
- **Navigation Mode**: Execute ground station commands
- **Manual Mode**: Direct gimbal control (placeholder)

## Core Functionality

### 1. Actuator Control
```c
// Position control with deadband
if (fabs(axis1_error) > TVC_DEADBAND) {
    if (axis1_error > 0.0) {
        Extend(1);  // Move actuator forward
    } else {
        Retract(1); // Move actuator backward
    }
} else {
    Hold(1);        // Stop movement
}
```

**Key Parameters:**
- `TVC_DEADBAND = 25.0` ADC units
- `TVC_AXIS1_CENTER = 2300` ADC units (pitch center)
- `TVC_AXIS2_CENTER = 2300` ADC units (roll center)
- `TVC_MAX_ANGLE_DEG = ±5.5°` (physical gimbal limits)

### 2. Angle Conversion
**Linear to Angular:**
```c
angular = (ADC_value - center_position) / conversion_factor
```

**Angular to Linear:**
```c
ADC_value = (angle * conversion_factor) + center_position
```

**Conversion Factors:**
- Axis 1 (Pitch): 136.96 ADC units per degree
- Axis 2 (Roll): 123.0 ADC units per degree

### 3. Attitude Compensation
**Purpose**: Counteract vehicle pitch/roll to maintain stable gimbal pointing

**Algorithm:**
```c
// Calculate deviation from level
float roll_deviation = tvc_roll_deg - 0.0f;    // Target: 0° roll
float pitch_deviation = tvc_pitch_deg - 0.0f;  // Target: 0° pitch

// Apply deadband to prevent jittery control
if (fabs(roll_deviation) > TVC_DEADBAND_DEG) {
    attitude_compensation_roll = -roll_deviation * TVC_COMPENSATION_GAIN;
}

// Limit compensation to prevent excessive gimbal movement
if (attitude_compensation_roll > TVC_MAX_COMPENSATION_DEG) {
    attitude_compensation_roll = TVC_MAX_COMPENSATION_DEG;
}
```

**Parameters:**
- `TVC_COMPENSATION_GAIN = 1.0f` (adjustable)
- `TVC_DEADBAND_DEG = 0.5°` (prevents jitter)
- `TVC_MAX_COMPENSATION_DEG = 3.0°` (safety limit)

### 4. GPS Drift Compensation
**Purpose**: Detect and correct for unwanted vehicle drift using sensor fusion

**Sensor Fusion:**
```c
// Prioritize IMU for short-term, GPS for long-term
float imu_weight = 0.7f;  // Higher weight for IMU (more responsive)
float gps_weight = 0.3f;  // Lower weight for GPS (more stable)

if (gps_valid) {
    fused_velocity = imu_weight * imu_velocity + gps_weight * gps_velocity;
} else {
    fused_velocity = imu_velocity;  // Fallback to IMU only
}
```

**Physics-Based Compensation:**
```c
// Calculate desired velocity from gimbal angle and throttle
CalculateDesiredVelocity(gimbal_pitch, gimbal_roll, throttle_percent, 
                        &desired_velocity_north, &desired_velocity_east);

// Calculate velocity error (drift)
velocity_error = fused_velocity - desired_velocity;

// Apply compensation to gimbal angles
pitch_compensation = -velocity_error_north * compensation_gain;
roll_compensation = -velocity_error_east * compensation_gain;
```

### 5. Velocity Calculation
**IMU Integration**: Calculate velocity from acceleration data
```c
// Simple integration: v = v0 + a*dt
tvc_velocity_north_ms = prev_velocity + acc_north_ms2 * dt;

// Apply damping to prevent drift
tvc_velocity_north_ms *= damping_factor;  // 0.95f
```

**GPS Velocity**: Calculate from position changes using Haversine formula
```c
// Calculate distance and bearing between GPS positions
float distance_m = EARTH_RADIUS_M * c;
float bearing_rad = atan2f(y, x);

// Convert to velocity components
velocity_north = velocity_ms * cosf(bearing_rad);
velocity_east = velocity_ms * sinf(bearing_rad);
```

## Command System

### 1. Navigation Commands
**Format**: `"Move [Direction] [Distance]m [Velocity]ms"`
**Example**: `"Move Forward 5m 1ms"`

**Processing:**
```c
// Parse command components
strncpy(command_parser.direction, "Forward", sizeof(command_parser.direction));
command_parser.distance_m = 5.0f;
command_parser.velocity_ms = 1.0f;

// Calculate gimbal angle and duration
float gimbal_angle = CalculateGimbalAngle(velocity_ms, thrust_n);
float duration_s = distance_m / velocity_ms;
```

### 2. System State Commands
- `"system tvc enable"` - Enable TVC system
- `"system tvc disable"` - Disable TVC system
- `"system hover"` - Enter hover mode
- `"system rotate 30"` - Rotate yaw by 30 degrees

### 3. Physics-Based Navigation
**Gimbal Angle Calculation:**
```c
// Calculate required gimbal angle for desired velocity
float gimbal_angle = CalculateGimbalAngleFromVelocity(velocity_ms, direction_deg, thrust_n);
```

**Duration Calculation:**
```c
// Calculate movement duration based on distance and velocity
float duration_s = distance_m / velocity_ms;
```

**Thrust Offset Calculation:**
```c
// Calculate thrust offset for velocity control
float thrust_offset = CalculateThrustOffset(velocity_ms, acceleration_ms2, mass_kg);
```

## Data Flow

### 1. Input Data Sources
- **IMU**: `tvc_acc_x_g`, `tvc_acc_y_g`, `tvc_acc_z_g` (acceleration)
- **IMU**: `tvc_roll_deg`, `tvc_pitch_deg`, `tvc_yaw_deg` (attitude)
- **GPS**: `gps_latitude`, `gps_longitude`, `gps_velocity_*` (position/velocity)
- **Commands**: `command_string` from ground station

### 2. Processing Pipeline
1. **Filter ADC values** (low-pass filter, 2 taps)
2. **Update velocity** from IMU acceleration
3. **Check for commands** from ground station
4. **Calculate compensation** (attitude + GPS drift)
5. **Apply to gimbal angles** (pitch/roll)
6. **Convert to linear positions** (ADC values)
7. **Control actuators** (extend/retract/hold)

### 3. Output Control
- **Actuator Control**: GPIO pins PB12-PB15
- **Status Messages**: "Navigation Complete", "TVC Enabled", etc.
- **Telemetry**: Position, velocity, compensation angles

## Configuration Parameters

### Actuator Limits (update when system is rebuilt / calibrated)
```c
#define TVC_AXIS1_MIN_POSITION 1800    // Minimum ADC value
#define TVC_AXIS1_MAX_POSITION 2800    // Maximum ADC value
#define TVC_AXIS2_MIN_POSITION 1800    // Minimum ADC value
#define TVC_AXIS2_MAX_POSITION 2800    // Maximum ADC value
```

### Control Gains
```c
#define TVC_COMPENSATION_GAIN 1.0f     // Attitude compensation gain
#define TVC_DEADBAND_DEG 0.5f          // Attitude deadband
#define TVC_MAX_COMPENSATION_DEG 3.0f  // Maximum compensation angle
```

### Physics Parameters
```c
#define TVC_THRUST_TO_ANGLE_FACTOR 0.1f  // Thrust to angle conversion
#define MAX_THRUST_N 1000.0f             // Maximum thrust (Newtons)
#define VEHICLE_MASS_KG 50.0f            // Vehicle mass (kg)
```

## Safety Features

### 1. Angle Limits
- Physical gimbal limits: ±5.5°
- Software limits prevent excessive movement
- Clamping to actuator position limits

### 2. Data Validation
- GPS data timeout (5 seconds)
- IMU data validity checks
- Altitude cross-check with altimeter

### 3. Fallback Modes
- IMU-only operation if GPS fails
- Hold position if system disabled
- Command timeout and clearing

## Integration Points

### 1. Heartbeat System
- Receives commands via `command_string`
- Sends status messages via `status_message`
- Includes GPS data in telemetry

### 2. IMU System
- Thread-safe data access via dedicated TVC variables
- Real-time acceleration and attitude data
- Velocity calculation from acceleration integration

### 3. GPS System
- Position and velocity data for drift compensation
- Target position setting for navigation
- Cross-check with altimeter for altitude validation

## Control Algorithms (Placeholder Implementation)

### 1. Gimbal Angle Calculation
**Function**: `CalculateGimbalAngleFromVelocity()`
**Purpose**: Convert desired velocity to required gimbal angle
**Status**: **PLACEHOLDER** - Requires physical system measurements

**Dependencies for Implementation:**
- Thrust-to-velocity relationship
- Gimbal angle effectiveness
- Vehicle mass and drag characteristics
- Engine response characteristics

### 2. Thrust Offset Calculation
**Function**: `CalculateThrustOffset()`
**Purpose**: Calculate thrust adjustment for velocity control
**Status**: **PLACEHOLDER** - Requires physical system measurements

**Dependencies for Implementation:**
- Engine throttle response
- Thrust vs throttle relationship
- Vehicle drag characteristics
- Altitude and atmospheric conditions

### 3. Main Control Algorithm
**Function**: `ApplyControlAlgorithm()`
**Purpose**: Core TVC control logic
**Status**: **PLACEHOLDER** - Requires control system design

**Implementation Options:**
- PID control
- LQR (Linear Quadratic Regulator)
- Model Predictive Control (MPC)
- Adaptive control

## Gimbal IMU Integration (Implemented)

### 1. Hardware Requirements
- **X-axis IMU**: Measure gimbal pitch rotation
- **Y-axis IMU**: Measure gimbal roll rotation
- **Installation**: Mounted on gimbal inner stage
- **Purpose**: Direct measurement of actual thrust vector direction

### 2. Software Integration
**Module**: `ProcessGimbalIMU.c/h`
**Functions**: `updateGimbalIMUData()`, `getGimbalAngles()`, `getGimbalVelocities()`, `getGimbalAccelerations()`
**Status**: **IMPLEMENTED** - Ready for hardware installation

**Implementation Features:**
- Single IMU per axis (simplified from multi-IMU fusion)
- Dedicated UART buffers for each gimbal IMU
- Calibration system with offset calculation
- Angle wrapping and data validation
- Real-time data processing and filtering

### 3. Benefits
- **Direct measurement** of actual gimbal angles
- **Improved accuracy** over actuator position estimation
- **Real-time feedback** for control algorithms
- **Fault detection** for gimbal mechanism issues
- **Simplified architecture** compared to vehicle IMU system

## Future Enhancements

### 1. Advanced Control
- PID control for smoother actuator movement
- Kalman filtering for better sensor fusion
- Adaptive gain scheduling

### 2. Navigation Features
- Waypoint following
- Obstacle avoidance
- Return-to-home functionality

### 3. Safety Systems
- Emergency stop commands
- Fault detection and recovery
- Redundant sensor validation

## Testing and Validation

### 1. Unit Tests
- Angle conversion accuracy
- Actuator control response
- Command parsing validation

### 2. Integration Tests
- IMU data flow verification
- GPS drift compensation testing
- End-to-end command execution

### 3. Flight Tests
- Hover stability validation
- Navigation command accuracy
- Drift compensation effectiveness

---

**Last Updated**: January 2025  
**Version**: 1.0  
**Author**: Leapfrog Development Team
