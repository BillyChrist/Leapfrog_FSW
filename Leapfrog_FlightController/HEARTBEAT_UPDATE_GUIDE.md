# Heartbeat Package Update Guide

This guide explains how to add, remove, or modify telemetry fields in the LEAPFROG heartbeat system.

## Overview

The heartbeat system has three main components that must be kept in sync:
1. **STM32 Firmware** (`Leapfrog_Vehicle/Core/Src/heartbeat.c`) - Sends data
2. **ROS2 Message** (`src/flightcontrol/msg/Heartbeat.msg`) - Internal communication
3. **Protobuf Message** (`src/flightcontrol/heartbeat.proto`) - External communication

## Data Flow

```
STM32 → STM32_Bridge → ROS2 Heartbeat.msg → FlightManager → Protobuf → GroundStation
```

## Step-by-Step Update Process

### 1. Update STM32 Firmware (if adding new sensor data)

**File**: `Leapfrog_Vehicle/Core/Src/heartbeat.c`

#### Add to STM32Response struct (in `stm32_bridge.hpp`):
```c
typedef struct {
  // ... existing fields ...
  
  // NEW FIELD - Add your field here
  float32_t gps_latitude;     // Example: GPS latitude
  float32_t gps_longitude;    // Example: GPS longitude
} STM32Response;
```

#### Update `buildHeartbeatPacket()` function:
```c
void buildHeartbeatPacket(STM32Response *resp_pkt){
    // ... existing code ...
    
    // NEW FIELD - Add data population
    resp_pkt->gps_latitude = gps_lat_value;   // Get from your GPS sensor
    resp_pkt->gps_longitude = gps_lon_value;  // Get from your GPS sensor
}
```

#### Update `sendHeartbeatPacket()` function:
```c
void sendHeartbeatPacket(UART_HandleTypeDef *huart, STM32Response *resp_pkt){
    // ... existing code ...
    
    // NEW FIELD - Add to serial transmission
    emplace_buffer(tx_buf, &offset, &resp_pkt->gps_latitude, sizeof(float));
    emplace_buffer(tx_buf, &offset, &resp_pkt->gps_longitude, sizeof(float));
}
```

### 2. Update STM32 Bridge Header

**File**: `src/stm32_bridge/include/stm32_bridge.hpp`

#### Add to STM32Response struct:
```cpp
typedef struct {
  // ... existing fields ...
  
  // NEW FIELD - Must match STM32 firmware exactly
  float32_t gps_latitude;
  float32_t gps_longitude;
} STM32Response;
```

### 3. Update ROS2 Message Definition

**File**: `src/flightcontrol/msg/Heartbeat.msg`

#### Add your field:
```msg
# GPS Data
float32 gps_latitude
float32 gps_longitude
```

**Data Types Available:**
- `bool` - Boolean (true/false)
- `int8`, `int16`, `int32`, `int64` - Signed integers
- `uint8`, `uint16`, `uint32`, `uint64` - Unsigned integers
- `float32`, `float64` - Floating point numbers
- `string` - Text strings
- `time`, `duration` - ROS2 time types

### 4. Update STM32 Bridge Implementation

**File**: `src/stm32_bridge/src/stm32_bridge.cpp`

#### Add parsing in `timer_callback()`:
```cpp
// Parse GPS telemetry (add after existing parsing)
access_buffer(packet_bytes.data(), &offset, &packet.gps_latitude, sizeof(float));
access_buffer(packet_bytes.data(), &offset, &packet.gps_longitude, sizeof(float));
```

#### Add to telemetry message creation:
```cpp
// Create and publish telemetry message
auto telemetry_msg = flightcontrol::msg::Heartbeat();
// ... existing fields ...

// GPS Data
telemetry_msg.gps_latitude = packet.gps_latitude;
telemetry_msg.gps_longitude = packet.gps_longitude;
```

### 5. Update Protobuf Definition

**File**: `src/flightcontrol/heartbeat.proto`

#### Add to Heartbeat message:
```proto
message Heartbeat {
  // ... existing fields ...
  
  // GPS Data
  float gps_latitude = 27;    // Use next available field number
  float gps_longitude = 28;   // Use next available field number
}
```

**Important**: Field numbers must be unique and sequential. Check existing fields to find the next available number.

### 6. Update FlightManager (if needed)

**File**: `src/flightcontrol/src/flightmanager.cpp`

#### Update `SendProtobufHeartbeat()` function:
```cpp
void FlightManager::SendProtobufHeartbeat() {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    leapfrog::Heartbeat hb_msg;
    
    // ... existing fields ...
    
    // NEW FIELD - Add protobuf population
    hb_msg.set_gps_latitude(latest_telemetry_.gps_latitude);
    hb_msg.set_gps_longitude(latest_telemetry_.gps_longitude);
    
    // ... rest of function ...
}
```

### 7. Rebuild Packages

```bash
# Clean and rebuild
rm -rf build/ install/
colcon build --packages-select communication flightcontrol stm32_bridge
```

## Removing Fields

### To remove a field:

1. **Remove from STM32 firmware** - Delete from struct and functions
2. **Remove from ROS2 message** - Delete from `Heartbeat.msg`
3. **Remove from Protobuf** - Delete from `heartbeat.proto`
4. **Remove from STM32 bridge** - Delete parsing and assignment code
5. **Remove from FlightManager** - Delete from protobuf population
6. **Rebuild packages**

## Modifying Field Types

### To change a field type:

1. **Update STM32 firmware** - Change struct definition and serialization
2. **Update ROS2 message** - Change type in `Heartbeat.msg`
3. **Update Protobuf** - Change type in `heartbeat.proto`
4. **Update STM32 bridge** - Change parsing code if needed
5. **Update FlightManager** - Change protobuf population if needed
6. **Rebuild packages**

## Common Issues and Solutions

### 1. Compilation Errors
- **Field not found**: Check that field name matches exactly across all files
- **Type mismatch**: Ensure data types are consistent across all components
- **Missing field**: Verify field was added to all required locations

### 2. Runtime Issues
- **Data corruption**: Check that parsing order matches STM32 transmission order
- **Missing data**: Verify field is being populated in STM32 firmware
- **Wrong values**: Check data type conversions and byte ordering

### 3. Field Number Conflicts (Protobuf)
- **Duplicate field numbers**: Each field must have a unique number
- **Missing field numbers**: Use sequential numbers (1, 2, 3, etc.)

## Testing

### 1. Build Test
```bash
colcon build --packages-select communication flightcontrol stm32_bridge
```

### 2. Runtime Test
```bash
# Source workspace
source install/setup.bash

# Run nodes
ros2 run flightcontrol flightmanager /dev/ttyUSB0 115200 &
ros2 run stm32_bridge stm32_bridge /dev/ttyUSB0 115200 &

# Monitor topics
ros2 topic echo /telemetry
ros2 topic echo /heartbeat
```

### 3. Verify Data Flow
- Check that new fields appear in ROS2 topics
- Verify protobuf serialization works
- Confirm ground station receives new data

## Best Practices

1. **Always update all components** - Don't skip any step
2. **Use descriptive field names** - Make purpose clear
3. **Choose appropriate data types** - Don't waste bandwidth
4. **Test thoroughly** - Verify data integrity
5. **Document changes** - Update this guide if needed
6. **Version control** - Commit changes incrementally

## Example: Adding GPS Data

Here's a complete example of adding GPS latitude and longitude:

### STM32 Firmware Changes
```c
// In STM32Response struct
float32_t gps_latitude;
float32_t gps_longitude;

// In buildHeartbeatPacket()
resp_pkt->gps_latitude = get_gps_latitude();
resp_pkt->gps_longitude = get_gps_longitude();

// In sendHeartbeatPacket()
emplace_buffer(tx_buf, &offset, &resp_pkt->gps_latitude, sizeof(float));
emplace_buffer(tx_buf, &offset, &resp_pkt->gps_longitude, sizeof(float));
```

### ROS2 Message Changes
```msg
# GPS Data
float32 gps_latitude
float32 gps_longitude
```

### Protobuf Changes
```proto
message Heartbeat {
  // ... existing fields ...
  float gps_latitude = 27;
  float gps_longitude = 28;
}
```

### STM32 Bridge Changes
```cpp
// In parsing section
access_buffer(packet_bytes.data(), &offset, &packet.gps_latitude, sizeof(float));
access_buffer(packet_bytes.data(), &offset, &packet.gps_longitude, sizeof(float));

// In telemetry message creation
telemetry_msg.gps_latitude = packet.gps_latitude;
telemetry_msg.gps_longitude = packet.gps_longitude;
```

### FlightManager Changes
```cpp
// In SendProtobufHeartbeat()
hb_msg.set_gps_latitude(latest_telemetry_.gps_latitude);
hb_msg.set_gps_longitude(latest_telemetry_.gps_longitude);
```

This completes the GPS data addition example.
