/*===================================================================
 * Project    : LEAPFROG
 * File       : stm32_bridge.hpp
 * Author     : Julia Schatz (2020) | Maintained by BillyChrist (2025)
 * Description: Header file for the STM32Bridge ROS 2 node.
 *
 * Core Responsibilities:
 *  - Subscribe to ROS 2 heartbeat messages from the flight manager
 *  - Encode and send command packets to STM32 over serial
 *  - Receive structured telemetry packets from STM32 over serial
 *  - Parse and publish telemetry data to ROS 2 topics
 *  - Maintain a live telemetry log file for diagnostics (only in DEBUG mode)
 *  - Monitor for timeout from STM32 or heartbeat dropouts
 *  - All terminal output and file logging is suppressed in flight mode (DEBUG == false)
 *  - All warnings/errors are forwarded as telemetry in flight mode
 *
 * Serial Protocol:
 *  - Pi → STM32: Encodes system state into `STM32Message` struct
 *  - STM32 → Pi: Sends `STM32Response` packets containing telemetry
 *  - All packets are prefixed with sync bytes for framing
 *
 * Notes:
 *  - `STM32_RESPONSE_SIZE` and struct layout must exactly match firmware side
 *  - All sensors are now owned by the STM32 and telemetry is sent via heartbeat
 *
 *===================================================================*/

#pragma once

#include <memory>
#include <chrono>
#include <cstdint>
#include <fstream>

#include "Serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "flightcontrol/msg/heartbeat.hpp"

typedef float float32_t;
typedef std::chrono::steady_clock stClock;

#define STM32_RESPONSE_SIZE sizeof(STM32Response)
typedef struct {
  // IMU Data
  float32_t roll_deg;
  float32_t pitch_deg;
  float32_t yaw_deg;
  float32_t acc_x_g;
  float32_t acc_y_g;
  float32_t acc_z_g;
  float32_t angvel_x_degs;
  float32_t angvel_y_degs;
  float32_t angvel_z_degs;

  // TVC Data
  float32_t tvc_a_pos;
  float32_t tvc_b_pos;

  // Engine Data
  int32_t engine_turbine_rpm;
  int32_t engine_rpm_setpoint;
  int32_t engine_egt_c;
  float32_t engine_pump_voltage;
  uint8_t engine_turbine_state;
  uint8_t engine_off_condition;
  uint8_t engine_throttle_percent;
  float32_t engine_current_a;

  // Altitude Data
  int16_t altitude;

  // System Status
  uint8_t heartbeat_counter;
  uint8_t imu_calibration_status;
  // TODO: Add GPS messages
} STM32Response;

// Number of bytes in the packet going Pi->STM32
#define STM32_MESSAGE_SIZE sizeof(STM32Message)
// Protocol sync byte info. Must be synced with STM32
#define STM32_SYNC_BYTE_COUNT 5
#define STM32_SYNC_BYTE 0x77

typedef struct {
  uint8_t engine_state;
  uint8_t tvc_state;
  uint8_t acs_state;
  float32_t tvc_angle1;
  float32_t tvc_angle2;
  float32_t engine_hover_m;
  uint8_t engine_thrust_p;
  int8_t imu_calibrate;
  uint8_t engine_safe;
  uint8_t engine_power;
  uint8_t checksum;
  uint8_t safeland_command; // 0 = normal, 1 = SafeLand
} STM32Message;

class STM32Bridge : public rclcpp::Node, public Serial
{
public:
  STM32Bridge(std::string port, int baud);
    
  // \brief returns the time elapsed since the epoch time t0
  // \return number of seconds since epoch time t0
  float getTimeSinceEpoch();
    
  const stClock::time_point tEpoch = stClock::now();
  const uint8_t liveTelemUpdate_s = 5;
  const string liveTelemUpdate_file = "/home/ubuntu/vehicle/src/stm32_bridge/src/liveTelem.txt";

private:
  void heartbeat_callback(const flightcontrol::msg::Heartbeat::SharedPtr heartbeat);
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<flightcontrol::msg::Heartbeat>::SharedPtr heartbeat_sub;
  rclcpp::Publisher<flightcontrol::msg::Heartbeat>::SharedPtr telemetry_pub;

  // System state
  bool guidance_internal = false;
  bool enable_acs = false;
  bool enable_tvc = false;
  bool enable_engine = false;
  bool safe_engine = false;
  bool power_engine = false;
  float engine_thrust = 0.0f;
  float engine_hover = 0.0f;
  float tvc_angle1 = 0.0f;
  float tvc_angle2 = 0.0f;
  uint8_t imu_calibration_status = 0;

  // Timing
  std::chrono::time_point<std::chrono::system_clock> last_heartbeat;
  std::chrono::time_point<std::chrono::system_clock> last_stm32;
  const std::chrono::milliseconds stm32_timeout{1000};
    
  float lastLTU_s = -1*liveTelemUpdate_s;
  float thisLTU_s = 0;

  // Error flags for telemetry forwarding
  bool heartbeat_timeout_flag = false;
  bool packet_size_error_flag = false;
};

void emplace_buffer(uint8_t* buf, size_t* offset, void* data, size_t sz);
void access_buffer(uint8_t* buf, size_t* offset, void* data, size_t sz);

// test