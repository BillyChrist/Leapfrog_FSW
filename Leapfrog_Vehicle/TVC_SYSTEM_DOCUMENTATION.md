# TVC (Thrust Vector Control) System Documentation

## Overview
The TVC system provides precise thrust vector control for the Leapfrog vehicle through gimbal angle adjustments. It integrates IMU data, GPS positioning, and command-driven navigation to maintain vehicle stability and execute precise maneuvers.

## System Architecture

### Core Components
- **TVC Controller** (`tvc.c`): Main control loop running at 100Hz
- **IMU Integration** (`ProcessIMU.c`): Provides acceleration and attitude data
- **Gimbal IMU Integration** (`ProcessGimbalIMU.c`): Direct gimbal angle measurement
- **GPS Integration** (`gps.c`): Provides position, velocity, and drift compensation
- **Heartbeat Communication** (`heartbeat.c`): Command reception and status reporting
- **Actuator Control**: Direct GPIO control of linear actuators

### Hardware Interface
- **Actuators**: 2-axis linear actuators (Pitch/Roll)
- **Position Feedback**: Potentiometers via ADC (PA4, PA6) for linear position
- **Control Pins**: GPIO PB12-PB15 for actuator control
- **Vehicle IMU**: 3-axis accelerometer and gyroscope data
- **Gimbal IMUs**: Two 3-axis IMUs (one on X-axis gimbal stage, one on Y-axis gimbal stage)
- **GPS Module**: UART-based position and velocity data

### Sensor Architecture
- **ADC Position Feedback**: Potentiometers on linear actuators provide position feedback
- **Gimbal IMU Feedback**: Direct angle measurement from IMUs mounted on gimbal stages
- **Sensor Fusion**: Redundant closed-loop control using both position sources
- **Fault Detection**: Cross-validation between ADC and IMU readings for reliability

## Control Modes

### System States
| State | Description | Effect |
|-------|-------------|---------|
| `SystemTVC_Disable` | TVC system disabled | Actuators hold current position |
| `SystemTVC_Manual` | Manual gimbal control | Direct gimbal angle control |
| `SystemTVC_Automatic` | Automatic control | Attitude compensation + navigation |

### Operational Modes
- **Hover Mode**: Maintain level position with attitude compensation
- **Navigation Mode**: Execute ground station commands
- **Manual Mode**: Direct gimbal control (placeholder)

## Core Functionality

### Actuator Control
**Position Control with Deadband:**
```c
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

### Angle Conversion
**Linear to Angular:** `angular = (ADC_value - center_position) / conversion_factor`
**Angular to Linear:** `ADC_value = (angle * conversion_factor) + center_position`

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TVC_DEADBAND` | 25.0 ADC units | Position control deadband |
| `TVC_AXIS1_CENTER` | 2300 ADC units | Pitch center position |
| `TVC_AXIS2_CENTER` | 2300 ADC units | Roll center position |
| `TVC_MAX_ANGLE_DEG` | ±5.5° | Physical gimbal limits |
| Axis 1 (Pitch) | 136.96 ADC/deg | Conversion factor |
| Axis 2 (Roll) | 123.0 ADC/deg | Conversion factor |

### Attitude Compensation
**Purpose**: Counteract vehicle pitch/roll to maintain stable gimbal pointing

```c
// Calculate deviation from level
float roll_deviation = tvc_roll_deg - 0.0f;    // Target: 0° roll
float pitch_deviation = tvc_pitch_deg - 0.0f;  // Target: 0° pitch

// Apply deadband to prevent jittery control
if (fabs(roll_deviation) > TVC_DEADBAND_DEG) {
    attitude_compensation_roll = -roll_deviation * TVC_COMPENSATION_GAIN;
}
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TVC_COMPENSATION_GAIN` | 1.0f | Attitude compensation gain |
| `TVC_DEADBAND_DEG` | 0.5° | Attitude deadband |
| `TVC_MAX_COMPENSATION_DEG` | 3.0° | Maximum compensation angle |

### GPS Drift Compensation
**Purpose**: Detect and correct for unwanted vehicle drift using sensor fusion

```c
// Sensor fusion with weighted average
float imu_weight = 0.7f;  // Higher weight for IMU (more responsive)
float gps_weight = 0.3f;  // Lower weight for GPS (more stable)

if (gps_valid) {
    fused_velocity = imu_weight * imu_velocity + gps_weight * gps_velocity;
} else {
    fused_velocity = imu_velocity;  // Fallback to IMU only
}
```

### Sensor Fusion (ADC + Gimbal IMU)
**Purpose**: Redundant closed-loop control using both position feedback sources

```c
// Fuse position feedback from ADC and gimbal IMU sensors
void FusePositionFeedback(void) {
    float adc_weight = 0.6f;        // Weight for ADC position feedback
    float imu_weight = 0.4f;        // Weight for gimbal IMU data
    
    if (gimbal_imu_data_valid) {
        // Both sensors available - use weighted fusion
        fused_pitch_deg = (adc_weight * tvcVal1_angular) + (imu_weight * gimbal_imu_pitch_deg);
        fused_roll_deg = (adc_weight * tvcVal2_angular) + (imu_weight * gimbal_imu_roll_deg);
        
        // IMU provides direct velocity measurement
        fused_pitch_velocity_degs = gimbal_imu_pitch_velocity_degs;
        fused_roll_velocity_degs = gimbal_imu_roll_velocity_degs;
    } else {
        // Fallback: Use ADC position feedback only
        fused_pitch_deg = tvcVal1_angular;
        fused_roll_deg = tvcVal2_angular;
    }
}
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `adc_weight` | 0.6f | Weight for ADC position feedback |
| `imu_weight` | 0.4f | Weight for gimbal IMU data |
| `fused_pitch_deg` | Calculated | Fused gimbal pitch angle |
| `fused_roll_deg` | Calculated | Fused gimbal roll angle |

## Command System

### TVC Commands
| Command | Description | Effect |
|---------|-------------|---------|
| `tvc enable` | Enable TVC (automatic mode) | Allows automatic gimbal control and navigation |
| `tvc disable` | Disable TVC system | Actuators hold current position |
| `tvc manual` | Enable TVC (manual mode) | Allows manual gimbal control |
| `tvc automatic` | Enable TVC (automatic mode) | Enables automatic gimbal control |

### Navigation Commands
| Command | Description | Effect |
|---------|-------------|---------|
| `navigation hover` | Enter hover mode | Maintain current position with attitude compensation |
| `navigation move <direction> <distance>m <velocity>ms` | Execute movement | Move in specified direction |
| `navigation rotate <degrees>` | Rotate vehicle | Change vehicle heading by specified degrees |

**Movement Examples:**
- `navigation move Forward 5m 1ms` - Move forward 5 meters at 1 m/s
- `navigation move Backward 3m 0.5ms` - Move backward 3 meters at 0.5 m/s
- `navigation move Left 2m 1.5ms` - Strafe left 2 meters at 1.5 m/s
- `navigation move Right 1m 2ms` - Strafe right 1 meter at 2 m/s

**Alternative Direct Format:**
- `Move Forward 5m 1ms` - Direct movement command (no "navigation" prefix)

## Data Flow

### Input Data Sources
| Source | Variables | Description |
|--------|-----------|-------------|
| **Vehicle IMU** | `tvc_acc_x_g`, `tvc_acc_y_g`, `tvc_acc_z_g` | Acceleration data |
| **Vehicle IMU** | `tvc_roll_deg`, `tvc_pitch_deg`, `tvc_yaw_deg` | Attitude data |
| **ADC Position** | `tvcVal1_angular`, `tvcVal2_angular` | Linear actuator position feedback |
| **Gimbal IMUs** | `gimbal_pitch_deg`, `gimbal_roll_deg` | Direct gimbal angles |
| **Gimbal IMUs** | `gimbal_pitch_velocity_degs`, `gimbal_roll_velocity_degs` | Gimbal angular velocities |
| **Fused Data** | `fused_pitch_deg`, `fused_roll_deg` | Sensor-fused position feedback |
| **GPS** | `gps_latitude`, `gps_longitude`, `gps_velocity_*` | Position/velocity |
| **Commands** | `command_string` | Ground station commands |

### Processing Pipeline
1. **Filter ADC values** (low-pass filter, 2 taps)
2. **Update gimbal IMU data** (direct angle measurement)
3. **Fuse position feedback** (ADC + gimbal IMU sensor fusion)
4. **Update velocity** from IMU acceleration
5. **Check for commands** from ground station
6. **Calculate compensation** (attitude + GPS drift)
7. **Apply to gimbal angles** (pitch/roll using fused data)
8. **Convert to linear positions** (ADC values)
9. **Control actuators** (extend/retract/hold with fused feedback)

### Output Control
- **Actuator Control**: GPIO pins PB12-PB15
- **Status Messages**: "Navigation Complete", "TVC Enabled", etc.
- **Telemetry**: Position, velocity, compensation angles

## Configuration Parameters

### Actuator Limits
| Parameter | Value | Description |
|-----------|-------|-------------|
| `TVC_AXIS1_MIN_POSITION` | 1800 ADC units | Minimum pitch position |
| `TVC_AXIS1_MAX_POSITION` | 2800 ADC units | Maximum pitch position |
| `TVC_AXIS2_MIN_POSITION` | 1800 ADC units | Minimum roll position |
| `TVC_AXIS2_MAX_POSITION` | 2800 ADC units | Maximum roll position |

### Control Gains
| Parameter | Value | Description |
|-----------|-------|-------------|
| `TVC_COMPENSATION_GAIN` | 1.0f | Attitude compensation gain |
| `TVC_DEADBAND_DEG` | 0.5° | Attitude deadband |
| `TVC_MAX_COMPENSATION_DEG` | 3.0° | Maximum compensation angle |

### Physics Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `TVC_THRUST_TO_ANGLE_FACTOR` | 0.1f | Thrust to angle conversion |
| `MAX_THRUST_N` | 1000.0f | Maximum thrust (Newtons) |
| `VEHICLE_MASS_KG` | 50.0f | Vehicle mass (kg) |

## Safety Features

### Angle Limits
- **Physical gimbal limits**: ±5.5°
- **Software limits**: Prevent excessive movement
- **Actuator clamping**: Enforce position limits

### Data Validation
- **GPS timeout**: 5 seconds
- **IMU validity checks**: Data quality validation
- **Altitude cross-check**: Altimeter validation

### Fallback Modes
- **IMU-only operation**: If GPS fails
- **Hold position**: If system disabled
- **Command timeout**: Automatic command clearing

## Integration Points

### Heartbeat System
- **Command reception**: Via `command_string`
- **Status reporting**: Via `status_message`
- **Telemetry**: GPS data included

### IMU System
- **Thread-safe access**: Dedicated TVC variables
- **Real-time data**: Acceleration and attitude
- **Velocity calculation**: From acceleration integration

### GPS System
- **Drift compensation**: Position and velocity data
- **Navigation targets**: Position setting
- **Altitude validation**: Cross-check with altimeter

## Gimbal IMU Integration

### Hardware Requirements
- **X-axis IMU**: Measure gimbal pitch rotation
- **Y-axis IMU**: Measure gimbal roll rotation
- **Installation**: Mounted on gimbal inner stage
- **Purpose**: Direct measurement of actual thrust vector direction

### Software Integration
**Module**: `ProcessGimbalIMU.c/h`
**Functions**: `updateGimbalIMUData()`, `getGimbalAngles()`, `getGimbalVelocities()`, `getGimbalAccelerations()`
**Status**: **IMPLEMENTED** - Ready for hardware installation

**Implementation Features:**
- Single IMU per axis (simplified from multi-IMU fusion)
- Dedicated UART buffers for each gimbal IMU
- Calibration system with offset calculation
- Angle wrapping and data validation
- Real-time data processing and filtering

### Benefits
- **Direct measurement** of actual gimbal angles
- **Improved accuracy** over actuator position estimation
- **Real-time feedback** for control algorithms
- **Fault detection** for gimbal mechanism issues
- **Simplified architecture** compared to vehicle IMU system

## Control Algorithms (Placeholder Implementation)

### Gimbal Angle Calculation
**Function**: `CalculateGimbalAngleFromVelocity()`
**Purpose**: Convert desired velocity to required gimbal angle
**Status**: **PLACEHOLDER** - Requires physical system measurements

**Dependencies:**
- Thrust-to-velocity relationship
- Gimbal angle effectiveness
- Vehicle mass and drag characteristics
- Engine response characteristics

### Thrust Offset Calculation
**Function**: `CalculateThrustOffset()`
**Purpose**: Calculate thrust adjustment for velocity control
**Status**: **PLACEHOLDER** - Requires physical system measurements

**Dependencies:**
- Engine throttle response
- Thrust vs throttle relationship
- Vehicle drag characteristics
- Altitude and atmospheric conditions

### Main Control Algorithm
**Function**: `ApplyControlAlgorithm()`
**Purpose**: Core TVC control logic
**Status**: **PLACEHOLDER** - Requires control system design

**Implementation Options:**
- PID control
- LQR (Linear Quadratic Regulator)
- Model Predictive Control (MPC)
- Adaptive control

---

**Last Updated**: January 2025  
**Version**: 1.0  
**Author**: BillyChrist
