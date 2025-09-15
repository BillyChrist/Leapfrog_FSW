# LEAPFROG Communication Protocol Documentation

## Overview
This document describes the complete communication protocol between the Leapfrog Ground Station, Flight Controller (Raspberry Pi), and STM32 Vehicle. The system uses a multi-layer approach with Protobuf for ground station communication and custom binary protocols for Pi-STM32 communication.

## Data Flow Architecture

```
Ground Station ←→ RFD900x Modem ←→ Flight Controller (Pi) ←→ UART ←→ STM32 Vehicle
     ↓                    ↓                    ↓                    ↓
  Protobuf            Protobuf            Custom Binary        Custom Binary
  (JSON-like)         (Binary)            (84 bytes)          (84 bytes)
```

## 1. Ground Station ↔ Flight Controller (Raspberry Pi)

### Protocol: Protobuf over RFD900x Modem

#### 1.1 Incoming Heartbeat (Pi → Ground Station)
**Message Type:** `leapfrog.Heartbeat`

```protobuf
message Heartbeat {
  // IMU Data
  float roll_deg = 1;
  float pitch_deg = 2;
  float yaw_deg = 3;
  float acc_x_g = 4;
  float acc_y_g = 5;
  float acc_z_g = 6;
  float angvel_x_degs = 7;
  float angvel_y_degs = 8;
  float angvel_z_degs = 9;

  // TVC Data
  float tvc_a_pos = 10;
  float tvc_b_pos = 11;

  // Engine Telemetry
  int32 engine_turbine_rpm = 12;
  int32 engine_rpm_setpoint = 13;
  int32 engine_egt_c = 14;
  float engine_pump_voltage = 15;
  uint32 engine_turbine_state = 16;
  uint32 engine_off_condition = 17;
  uint32 engine_throttle_percent = 18;
  float engine_current_a = 19;

  // Altitude
  int32 altitude = 20;

  // System Status
  uint32 heartbeat_counter = 21;
  bool guidance_internal = 22;
  bool enable_acs = 23;
  bool enable_tvc = 24;
  bool enable_engine = 25;
  bool imu_calibration_status = 26;
  
  // Navigation command and status (NEW)
  string command_string = 27;    // Navigation command from ground station (e.g., "Move Forward 5m 1ms")
  string status_message = 28;    // Status message from STM32 (e.g., "Navigation Complete")
}
```

#### 1.2 Outgoing Commands (Ground Station → Pi)
**Message Type:** `leapfrog.Command`

```protobuf
message Command {
  string command_text = 1; // e.g. "navigation move Forward 5m 1ms"
  // Optionally, add more structured fields for advanced commands
}
```

### 1.3 Command Structure for Ground Station

#### Direct Subsystem Commands
**Purpose:** Enable/disable vehicle subsystems

| Command | Description | Effect |
|---------|-------------|---------|
| `tvc enable` | Enable TVC (automatic mode) | Allows automatic gimbal control and navigation |
| `tvc disable` | Disable TVC system | Actuators hold current position |
| `tvc manual` | Enable TVC (manual mode) | Allows manual gimbal control |
| `tvc automatic` | Enable TVC (automatic mode) | Enables automatic gimbal control |
| `acs enable` | Enable ACS system | Enables attitude control and stabilization |
| `acs disable` | Disable ACS system | Disables attitude control |
| `engine enable` | Enable engine (automatic mode) | Enables thrust generation |
| `engine disable` | Disable engine | Disables thrust generation |

#### Navigation Commands
**Purpose:** Control vehicle movement and positioning

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

**Rotation Examples:**
- `navigation rotate 30` - Rotate 30° clockwise from current heading
- `navigation rotate -45` - Rotate 45° counter-clockwise from current heading
- `navigation rotate 180` - Turn around (180° rotation)

**Alternative Direct Format:**
- `Move Forward 5m 1ms` - Direct movement command (no "navigation" prefix)

## 2. Flight Controller (Pi) ↔ STM32 Vehicle

### Protocol: Custom Binary over UART2

#### 2.1 Pi → STM32 Message (Commands)
**Size:** 84 bytes total
**Structure:** 5 sync bytes + 79 message bytes

```
Byte Layout:
[0-4]   : Sync bytes (0x77, 0x77, 0x77, 0x77, 0x77)
[5-8]   : engine_state (uint8_t)
[9-12]  : tvc_state (uint8_t)
[13-16] : acs_state (uint8_t)
[17-20] : tvc_angle1_mdeg (int32_t) - TVC angle 1 in millidegrees
[21-24] : tvc_angle2_mdeg (int32_t) - TVC angle 2 in millidegrees
[25-28] : engine_hover_mm (int32_t) - Engine hover height in millimeters
[29-32] : engine_thrust_p (uint8_t)
[33-36] : calibrateIMU (int8_t)
[37-40] : engine_safe (uint8_t)
[41-44] : engine_power (uint8_t)
[45-108]: command_string (char[64]) - Navigation command string
[109-112]: checksum (uint8_t) - Sum of bytes 5-108
```

#### 2.2 STM32 → Pi Message (Telemetry)
**Size:** Variable (depends on STM32Response structure)
**Structure:** 5 sync bytes + telemetry data

```
Byte Layout:
[0-4]   : Sync bytes (0x77, 0x77, 0x77, 0x77, 0x77)
[5-8]   : roll_deg (float)
[9-12]  : pitch_deg (float)
[13-16] : yaw_deg (float)
[17-20] : acc_x_g (float)
[21-24] : acc_y_g (float)
[25-28] : acc_z_g (float)
[29-32] : angvel_x_degs (float)
[33-36] : angvel_y_degs (float)
[37-40] : angvel_z_degs (float)
[41-44] : tvc_a_pos (float)
[45-48] : tvc_b_pos (float)
[49-52] : engine_turbine_rpm (int32_t)
[53-56] : engine_rpm_setpoint (int32_t)
[57-60] : engine_egt_c (int32_t)
[61-64] : engine_pump_voltage (float)
[65-68] : engine_turbine_state (uint8_t)
[69-72] : engine_off_condition (uint8_t)
[73-76] : engine_throttle_percent (uint8_t)
[77-80] : engine_current_a (float)
[81-84] : altitude (int16_t)
[85-88] : heartbeat_counter (uint8_t)
[89-92] : imu_calibration_status (uint8_t)
[93-156]: status_message (char[64]) - Status message from STM32
[157-160]: checksum (uint8_t) - Sum of all data bytes
```

## 3. System States and Control

### 3.1 System State Hierarchy

The LEAPFROG system operates with subsystem-specific state management:

```
Subsystem States:
├── TVC States:
│   ├── SystemTVC_Disable    - TVC disabled, actuators hold position
│   ├── SystemTVC_Manual     - TVC in manual control mode
│   └── SystemTVC_Automatic  - TVC in automatic control mode
├── ACS States:
│   ├── SystemACS_Disable    - ACS disabled, no attitude control
│   └── SystemACS_Enable     - ACS active, attitude control enabled
└── Engine States (Jet):
    ├── SystemJet_Disable    - Engine disabled, no thrust
    ├── SystemJet_Manual     - Engine in manual control mode
    └── SystemJet_Automatic  - Engine in automatic control mode
```

### 3.2 Yaw Pointing System

The system includes a `yaw_pointing` variable that tracks the vehicle's desired heading orientation:

```c
// Yaw pointing variable (degrees)
float yaw_pointing = 0.0f;  // Current desired heading

// Usage in ACS PID:
float setpoint_yaw = yaw_pointing + 0.0f;  // Target yaw angle
```

**Yaw Pointing Behavior:**
1. **Command Received:** `navigation rotate 30`
2. **Yaw Update:** `yaw_pointing += 30.0f`
3. **ACS Control:** ACS PID adjusts vehicle heading to `yaw_pointing`
4. **Status:** Vehicle rotates to new heading and maintains it

### 3.3 Navigation Command Flow

```
Ground Station → Flight Controller → STM32 → TVC System
     ↓                    ↓              ↓         ↓
Send Command        Parse & Forward   Process   Execute
"navigation move"   to STM32         Command   Movement
```

**Command Processing Steps:**
1. **Ground Station** sends: `navigation move Forward 5m 1ms`
2. **Flight Controller** parses and stores in `current_command_string`
3. **STM32Bridge** sends command to STM32 via UART2
4. **STM32** receives and processes via `ProcessIncomingCommand()`
5. **TVC System** executes navigation command
6. **STM32** sends status back: `"Navigation Complete"`

## 4. Command Parsing (STM32 Side)

#### 4.1 Navigation Commands
```c
// Expected format: "Move [Direction] [Distance]m [Velocity]ms"
// Example: "Move Forward 5m 1ms"

// Parsed into CommandParser structure:
typedef struct {
  char raw_command[64];      // "Move Forward 5m 1ms"
  char direction[16];        // "Forward"
  float distance_m;          // 5.0
  float velocity_ms;         // 1.0
  float gimbal_angle_deg;    // Calculated by TVC system
  float duration_s;          // Calculated: distance/velocity
  float jetThrottle_offset;  // Calculated throttle offset
  bool new_command;          // Flag for new command received
  bool executing;            // Flag for command currently executing
  uint32_t start_time_ms;    // Start time of command execution
} CommandParser;
```

#### 4.2 Subsystem State Commands
```c
// Subsystem state enums
typedef enum {
  SystemTVC_Disable = 0,
  SystemTVC_Manual,
  SystemTVC_Automatic
} TVC_State;

typedef enum {
  SystemACS_Disable = 0,
  SystemACS_Enable
} ACS_State;

typedef enum {
  SystemJet_Disable = 0,
  SystemJet_Manual,
  SystemJet_Automatic
} Jet_State;

// Command parsing examples:
// "tvc enable"         → tvcState = SystemTVC_Automatic
// "tvc manual"         → tvcState = SystemTVC_Manual
// "tvc disable"        → tvcState = SystemTVC_Disable
// "acs enable"         → acsState = SystemACS_Enable
// "acs disable"        → acsState = SystemACS_Disable
// "engine enable"      → engineState = SystemJet_Automatic
// "engine disable"     → engineState = SystemJet_Disable
// "navigation hover"   → calls Hover() function
// "navigation rotate 30" → yaw_pointing += 30.0f
```

#### 4.3 Command Processing Flow
```c
void ProcessIncomingCommand(const char* command_string) {
    // Parse command type
    if (strncmp(command_string, "navigation ", 11) == 0) {
        // Navigation commands: "navigation hover", "navigation move Forward 5m 1ms"
        ParseNavigationCommand(command_string + 11);
    } else if (strncmp(command_string, "tvc ", 4) == 0) {
        // Direct TVC commands: "tvc enable", "tvc disable", "tvc manual"
        ParseTVCCommand(command_string + 4);
    } else if (strncmp(command_string, "acs ", 4) == 0) {
        // Direct ACS commands: "acs enable", "acs disable"
        ParseACSCommand(command_string + 4);
    } else if (strncmp(command_string, "engine ", 7) == 0) {
        // Direct engine commands: "engine enable", "engine disable"
        ParseEngineCommand(command_string + 7);
    } else if (strncmp(command_string, "Move ", 5) == 0) {
        // Direct navigation command: "Move Forward 5m 1ms"
        ParseNavigationCommand(command_string);
    } else {
        SetStatusMessage("Invalid Command Format");
    }
}

void ParseTVCCommand(const char* cmd) {
    if (strcmp(cmd, "enable") == 0) {
        tvcState = SystemTVC_Automatic;
        SetStatusMessage("TVC (Auto Mode) Enabled");
    } else if (strcmp(cmd, "disable") == 0) {
        tvcState = SystemTVC_Disable;
        SetStatusMessage("TVC Disabled");
    } else if (strcmp(cmd, "manual") == 0) {
        tvcState = SystemTVC_Manual;
        SetStatusMessage("TVC (Manual Mode) Enabled");
    }
}
```

### 4.4 TVC Navigation Logic
- **Forward/Backward:** Adjusts Y-axis gimbal angle
- **Left/Right:** Adjusts X-axis gimbal angle
- **Duration:** Calculated as `distance_m / velocity_ms`
- **Gimbal Angle:** Calculated based on desired velocity and current thrust
- **Throttle Offset:** Dummy variable for future engine integration

## 5. Status Messages

### 5.1 Status Message Flow
1. **STM32** generates status: `"Navigation Complete"`, `"Command Executing"`, etc.
2. **STM32** sends status in telemetry packet
3. **Flight Controller** receives and forwards to ground station
4. **Ground Station** displays status to operator

### 5.2 Status Message Types

#### 5.2.1 System Status Messages
- `"System Ready"` - Default system status
- `"Navigation Hover Mode"` - Vehicle in hover mode
- `"Command Executing"` - Navigation command currently running
- `"Navigation Complete"` - Navigation command finished executing

#### 5.2.2 Subsystem Status Messages
- `"TVC (Auto Mode) Enabled"` - TVC system activated in automatic mode
- `"TVC (Manual Mode) Enabled"` - TVC system activated in manual mode
- `"TVC Disabled"` - TVC system deactivated, actuators holding position
- `"ACS Enabled"` - Attitude control system activated
- `"ACS Disabled"` - Attitude control system deactivated
- `"Engine (Auto Mode) Enabled"` - Engine system activated in automatic mode
- `"Engine Disabled"` - Engine system deactivated

#### 5.2.3 Navigation Status Messages
- `"Command Received"` - Navigation command received and parsed
- `"Navigation Complete"` - Navigation command finished executing
- `"Command Executing"` - Navigation command currently running
- `"Rotation Complete"` - Vehicle rotation finished
- `"Invalid Command Format"` - Command parsing failed

#### 5.2.4 Error Status Messages
- `"Invalid Command Format"` - Command parsing failed
- `"System Not Ready"` - System not ready for commands
- `"TVC Not Available"` - TVC system not available for navigation
- `"Engine Not Available"` - Engine not available for movement

## 6. Error Handling

### 6.1 UART Communication Errors
- **Sync Byte Mismatch:** Reset reception state
- **Checksum Failure:** Discard packet, log error
- **Timeout:** Reset to idle state, send error status

### 6.2 Command Processing Errors
- **Invalid Format:** Send `"Invalid Command Format"` status
- **System Disabled:** Ignore commands, send appropriate status
- **Execution Failure:** Send error status, return to hover

## 7. Implementation Notes

### 7.1 Ground Station Updates Required
1. Update `heartbeat.proto` to include `command_string` and `status_message` fields
2. Modify GUI to send navigation commands in new format
3. Add status message display to telemetry view
4. Update command parsing to handle new navigation commands

### 7.2 Flight Controller Updates Required
1. Update `CommandParser` to handle navigation commands
2. Modify `STM32Bridge` to include command_string in UART messages
3. Update `FlightManager` to store and forward navigation commands
4. Add status message handling in telemetry response

### 7.3 STM32 Updates Required
1. Add UART2 reception for Pi commands (COMPLETED)
2. Update message unpacking to handle command_string (COMPLETED)
3. Implement navigation command processing (COMPLETED)
4. Add status message generation and transmission (COMPLETED)

## 8. Testing Protocol

### 8.1 Unit Tests
- Test command parsing with various input formats
- Test UART message packing/unpacking
- Test checksum calculation and validation
- Test status message generation

### 8.2 Integration Tests
- Test complete command flow from ground station to STM32
- Test status message flow from STM32 to ground station
- Test error handling and recovery
- Test concurrent command processing

### 8.3 System Tests
- Test with actual RFD900x modem communication
- Test with real vehicle hardware
- Test under various network conditions
- Test with multiple rapid commands

---

**Document Version:** 1.0  
**Last Updated:** January 2025  
**Author:** Billy Christ
