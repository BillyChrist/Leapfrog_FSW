/*===================================================================
 * Project    : LEAPFROG
 * File       : stm32_bridge.cpp
 * Author     : Julia Schatz (2020) | Maintained by BillyChrist (2025)
 * Description: Implementation file for the STM32Bridge ROS 2 node.
 *
 * This class bridges communication between the Raspberry Pi and the STM32 MCU.
 *
 * Core Responsibilities:
 *  - Subscribe to ROS 2 heartbeat messages from the flight manager
 *  - Encode and send command packets to STM32 over serial
 *  - Receive structured telemetry packets from STM32 over serial
 *  - Parse and publish telemetry data to ROS 2 topics
 *  - Maintain a live telemetry log file for diagnostics
 *  - Monitor for timeout from STM32 or heartbeat dropouts
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

#include <chrono>
#include <memory>

#include "stm32_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr bool DEBUG = true; // Set to false for flight mode (disables print strings to FC terminal)

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef std::chrono::steady_clock stClock;

const std::chrono::milliseconds heartbeat_timeout{500};
const std::chrono::milliseconds stm32_timeout{500};

// Add error flag variables at the top of the file
static bool heartbeat_timeout_flag = false;
static bool packet_size_error_flag = false;
static bool stm32_timeout_flag = false;

void emplace_uint32(std::vector<uint8_t>& buffer, int begin, uint32_t u) {
  // Example: 01000010011100100110000101100100 -> [01000010, 01110010, 01100001, 01100100]
  buffer[begin] = u >> 0;
  buffer[begin+1] = u >> 8;
  buffer[begin+2] = u >> 16;
  buffer[begin+3] = u >> 24;
}

// STM32Bridge constructor: set up ROS2 subscriptions, publishers, and timer
STM32Bridge::STM32Bridge(std::string port, int baud) : Node("stm32_bridge"), Serial(port, baud, '\n', 100, STM32_RESPONSE_SIZE+STM32_SYNC_BYTE_COUNT) {
  // Subscribe to heartbeat topic from flight manager
  heartbeat_sub = this->create_subscription<flightcontrol::msg::Heartbeat>("heartbeat", 10, std::bind(&STM32Bridge::heartbeat_callback, this, _1));
  // Publish telemetry topic
  telemetry_pub = this->create_publisher<flightcontrol::msg::Heartbeat>("telemetry", 10);
  // Timer for fixed-rate telemetry and comms
  timer_ = this->create_wall_timer(
      50ms, std::bind(&STM32Bridge::timer_callback, this));
}

float STM32Bridge::getTimeSinceEpoch() {
    stClock::time_point t1 = stClock::now();
    std::chrono::duration<float> time_span = std::chrono::duration_cast<std::chrono::duration<float>>(t1 - tEpoch);
    return time_span.count();
}

// timer_callback: main loop for serial comms and telemetry
void STM32Bridge::timer_callback() {
  // Check for heartbeat timeout and set error flag
  static bool heartbeat_sticky = false;
  static bool safeland_active = false;
  if (std::chrono::system_clock::now() - last_heartbeat > heartbeat_timeout) {
    if (!heartbeat_sticky) {
      heartbeat_sticky = true;
      if (DEBUG) {
        RCLCPP_FATAL(this->get_logger(), "Heartbeat timeout");
      }
    }
    heartbeat_timeout_flag = true;
    safeland_active = true;
  } else {
    if (heartbeat_sticky) {
      if (DEBUG) {
        RCLCPP_FATAL(get_logger(), "Heartbeat stopped timing out");
      }
      heartbeat_sticky = false;
    }
    heartbeat_timeout_flag = false;
    safeland_active = false;
  }

  // Build STM32Message buffer (add safeland_command field)
  uint8_t acs_state = enable_acs ? 2 : 0; 
  uint8_t tvc_state = enable_tvc ? (guidance_internal ? 2 : 1) : 0; 
  uint8_t engine_state = enable_engine ? (guidance_internal ? 2 : 1) : 0; 
  uint8_t engine_safe = safe_engine ? 1 : 0; 
  uint8_t engine_power = power_engine ? 1 : 0; 

  // Floats are annoying to send over the wire, so use units of milli-whatever and treat them like ints instead
  // This is doable because we know the rough sizes of these, if they were all ~1e-6 this would be useless, but they're all around 1e1 and milli- gives us enough precision
  int32_t tvc_angle1_mdeg = tvc_angle1 * 1000.0; 
  int32_t tvc_angle2_mdeg = tvc_angle2 * 1000.0; 

  int32_t engine_hover_mm = engine_hover * 1000.0; // Originally uint32_t, but we may need to input a negative height for a landing procedure
  uint8_t engine_thrust_p = engine_thrust; 
  
  auto buffer = std::vector<uint8_t>(STM32_MESSAGE_SIZE); 
  buffer[0] = engine_state; 
  buffer[1] = tvc_state; 
  buffer[2] = acs_state; 
  emplace_uint32(buffer, 3, tvc_angle1_mdeg); 
  emplace_uint32(buffer, 7, tvc_angle2_mdeg); 
  emplace_uint32(buffer, 11, engine_hover_mm); 
  buffer[15] = engine_thrust_p; 
  buffer[16] = imu_calibration_status; 
  buffer[17] = engine_safe; 
  buffer[18] = engine_power; 
  buffer[19] = safeland_active ? 1 : 0; // safeland_command field

  // Calculate checksum
  uint8_t checksum = 0;     // Initialized to zero to avoid undefined values
  for (unsigned int i = 0; i < STM32_MESSAGE_SIZE-1; ++i) {
    checksum += buffer[i];
  }
  buffer[STM32_MESSAGE_SIZE-1] = checksum;

  // Send sync bytes and message
  auto sync_buffer = std::vector<uint8_t>(STM32_SYNC_BYTE_COUNT);
  std::fill(sync_buffer.begin(), sync_buffer.end(), STM32_SYNC_BYTE);
  Send(sync_buffer, false);
  Send(buffer, false);

  // Receive and parse telemetry from STM32
  auto receive_buffer = std::deque<uint8_t>();
  while (IsAvailable() > 0) {
    auto vec = Recv();
    for (uint8_t ch : vec) {
      receive_buffer.push_back(ch);
    }
  }
  // Sync byte and packet size check
  while (receive_buffer.size() > 0) {
    uint8_t rx_byte = receive_buffer.front();
    receive_buffer.pop_front();
    static int sync_byte_count = 0;
    static int remaining_to_rx = STM32_RESPONSE_SIZE;
    static std::vector<uint8_t> packet_bytes;
    if (sync_byte_count < STM32_SYNC_BYTE_COUNT) {
      if (rx_byte == STM32_SYNC_BYTE) {
        ++sync_byte_count;
      } else {
        sync_byte_count = 0;
      }
    } else {
      packet_bytes.push_back(rx_byte);
      --remaining_to_rx;
      if (remaining_to_rx == 0) {
        remaining_to_rx = STM32_RESPONSE_SIZE;
        sync_byte_count = 0;
        // Protocol robustness: check packet size
        if (packet_bytes.size() != STM32_RESPONSE_SIZE) {
          if (DEBUG) {
            RCLCPP_ERROR(this->get_logger(), "Invalid packet size: %zu", packet_bytes.size());
          }
          packet_size_error_flag = true;
          packet_bytes.clear();
          continue;
        } else {
          packet_size_error_flag = false;
        }
        last_stm32 = std::chrono::system_clock::now();
        STM32Response packet;
        size_t offset = 0;
        
        // Parse ACS telemetry
        access_buffer(packet_bytes.data(), &offset, &packet.roll_deg, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.pitch_deg, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.yaw_deg, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.acc_x_g, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.acc_y_g, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.acc_z_g, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.angvel_x_degs, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.angvel_y_degs, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.angvel_z_degs, sizeof(float));
        
        // Parse TVC telemetry
        access_buffer(packet_bytes.data(), &offset, &packet.tvc_a_pos, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.tvc_b_pos, sizeof(float));

        // // Parse Engine telemetry
        access_buffer(packet_bytes.data(), &offset, &packet.engine_turbine_rpm, sizeof(int32_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_rpm_setpoint, sizeof(int32_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_egt_c, sizeof(int32_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_pump_voltage, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_turbine_state, sizeof(uint8_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_off_condition, sizeof(uint8_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_throttle_percent, sizeof(uint8_t));
        access_buffer(packet_bytes.data(), &offset, &packet.engine_current_a, sizeof(float));
        access_buffer(packet_bytes.data(), &offset, &packet.altitude, sizeof(int16_t));
        
        // Parse Heartbeat telemetry
        access_buffer(packet_bytes.data(), &offset, &packet.heartbeat_counter, sizeof(uint8_t));

        // Parse Calibration Status
        access_buffer(packet_bytes.data(), &offset, &packet.imu_calibration_status, sizeof(uint8_t));
        // TODO TVC & GPS calibration
        
        packet_bytes.clear();

        // Create and publish telemetry message
        auto telemetry_msg = flightcontrol::msg::Heartbeat();
        
        // IMU Data
        telemetry_msg.roll_deg = packet.roll_deg;
        telemetry_msg.pitch_deg = packet.pitch_deg;
        telemetry_msg.yaw_deg = packet.yaw_deg;
        telemetry_msg.acc_x_g = packet.acc_x_g;
        telemetry_msg.acc_y_g = packet.acc_y_g;
        telemetry_msg.acc_z_g = packet.acc_z_g;
        telemetry_msg.angvel_x_degs = packet.angvel_x_degs;
        telemetry_msg.angvel_y_degs = packet.angvel_y_degs;
        telemetry_msg.angvel_z_degs = packet.angvel_z_degs;
        
        // TVC Data
        telemetry_msg.tvc_a_pos = packet.tvc_a_pos;
        telemetry_msg.tvc_b_pos = packet.tvc_b_pos;
        
        // Engine Data
        telemetry_msg.engine_turbine_rpm = packet.engine_turbine_rpm;
        telemetry_msg.engine_rpm_setpoint = packet.engine_rpm_setpoint;
        telemetry_msg.engine_egt_c = packet.engine_egt_c;
        telemetry_msg.engine_pump_voltage = packet.engine_pump_voltage;
        telemetry_msg.engine_turbine_state = packet.engine_turbine_state;
        telemetry_msg.engine_off_condition = packet.engine_off_condition;
        telemetry_msg.engine_throttle_percent = packet.engine_throttle_percent;
        telemetry_msg.engine_current_a = packet.engine_current_a;
        
        // Altitude Data
        telemetry_msg.altitude = packet.altitude;
        
        // System Status
        telemetry_msg.heartbeat_counter = packet.heartbeat_counter;
        telemetry_msg.guidance_internal = guidance_internal;
        telemetry_msg.enable_acs = enable_acs;
        telemetry_msg.enable_tvc = enable_tvc;
        telemetry_msg.enable_engine = enable_engine;
        telemetry_msg.imu_calibration_status = packet.imu_calibration_status;
        
        // Additional fields for compatibility
        telemetry_msg.tvc_angle1 = packet.tvc_a_pos;
        telemetry_msg.tvc_angle2 = packet.tvc_b_pos;
        telemetry_msg.engine_hover_height = engine_hover;
        telemetry_msg.safe_engine = safe_engine;
        telemetry_msg.power_engine = power_engine;
        telemetry_msg.engine_thrust = engine_thrust;
        // When publishing telemetry, add error flags to the message
        telemetry_msg.error_flags = 0;
        if (heartbeat_timeout_flag) telemetry_msg.error_flags |= 0x01;
        if (packet_size_error_flag) telemetry_msg.error_flags |= 0x02;
        if (stm32_timeout_flag)    telemetry_msg.error_flags |= 0x04;
        telemetry_pub->publish(telemetry_msg);

        // Update live telemetry log
        thisLTU_s = getTimeSinceEpoch();
        if (DEBUG && (thisLTU_s - lastLTU_s >= liveTelemUpdate_s)) {
          std::ofstream ltu(liveTelemUpdate_file);

          ltu << "System Status:" << endl;
          ltu << "  Guidance Internal: " << (guidance_internal ? "Yes" : "No") << endl;
          ltu << "  TVC Enabled: " << (enable_tvc ? "Yes" : "No") << endl;
          ltu << "  ACS Enabled: " << (enable_acs ? "Yes" : "No") << endl;
          ltu << "  Engine Enabled: " << (enable_engine ? "Yes" : "No") << endl;
          ltu << "  Engine Safe: " << (safe_engine ? "Yes" : "No") << endl;
          ltu << "  Engine Power: " << (power_engine ? "Yes" : "No") << endl;
          ltu << "  IMU Calibration Status: " << (int)packet.imu_calibration_status << endl;

          ltu << endl << "IMU Data:" << endl;
          ltu << "  Roll: " << packet.roll_deg << " deg" << endl;
          ltu << "  Pitch: " << packet.pitch_deg << " deg" << endl;
          ltu << "  Yaw: " << packet.yaw_deg << " deg" << endl;
          ltu << "  Acceleration (X,Y,Z): " << packet.acc_x_g << ", " << packet.acc_y_g << ", " << packet.acc_z_g << " g" << endl;
          ltu << "  Angular Velocity (X,Y,Z): " << packet.angvel_x_degs << ", " << packet.angvel_y_degs << ", " << packet.angvel_z_degs << " deg/s" << endl;

          ltu << endl << "TVC Data:" << endl;
          ltu << "  TVC A Position: " << packet.tvc_a_pos << " deg" << endl;
          ltu << "  TVC B Position: " << packet.tvc_b_pos << " deg" << endl;
          
          // TODO add GPS data
          
          ltu << endl << "Engine Data:" << endl;
          ltu << "  Turbine RPM: " << packet.engine_turbine_rpm << endl;
          ltu << "  RPM Setpoint: " << packet.engine_rpm_setpoint << endl;
          ltu << "  EGT: " << packet.engine_egt_c << " C" << endl;
          ltu << "  Pump Voltage: " << packet.engine_pump_voltage << " V" << endl;
          ltu << "  Turbine State: " << (int)packet.engine_turbine_state << endl;
          ltu << "  Off Condition: " << (int)packet.engine_off_condition << endl;
          ltu << "  Throttle: " << (int)packet.engine_throttle_percent << "%" << endl;
          ltu << "  Current: " << packet.engine_current_a << " A" << endl;

          ltu << endl << "Altitude Data:" << endl;
          ltu << "  Altitude: " << packet.altitude << " cm" << endl;

          // TODO Add GPS Data

          ltu << endl;
          ltu.close();

          lastLTU_s = thisLTU_s;
        }

        // Check for STM32 timeout
        static bool timeoutSticky = false;
        if (packet.heartbeat_counter == 0 && !timeoutSticky) {
          timeoutSticky = true;
          if (DEBUG) {
            RCLCPP_FATAL(this->get_logger(), "STM32 reported time out!");
          } else {
            // Forward error as telemetry (set a static error flag or publish warning)
            static bool stm32_timeout_flag = true;
          }
        }
        else if (packet.heartbeat_counter > 0 && timeoutSticky) {
          timeoutSticky = false;
          if (DEBUG) {
            RCLCPP_FATAL(this->get_logger(), "STM32 stopped reporting timeout");
          } else {
            // Forward error as telemetry (set a static error flag or publish warning)
            static bool stm32_timeout_flag = true;
          }
        }
      }
    }
  }

  // Check for STM32 communication timeout
  static bool stm32_sticky = false;
  if (std::chrono::system_clock::now() - last_stm32 > stm32_timeout) {
    if (!stm32_sticky) {
      stm32_sticky = true;
      if (DEBUG) {
        RCLCPP_WARN(this->get_logger(), "No message from STM32");
      } else {
        // Forward error as telemetry (set a static error flag or publish warning)
        static bool stm32_no_message_flag = true;
      }
    }
  }
  else if (stm32_sticky) {
    stm32_sticky = false;
    if (DEBUG) {
      RCLCPP_WARN(this->get_logger(), "Got message from STM32");
    } else {
      // Forward error as telemetry (set a static error flag or publish warning)
      static bool stm32_message_received_flag = true;
    }
  }
}

void STM32Bridge::heartbeat_callback(const flightcontrol::msg::Heartbeat::SharedPtr heartbeat) {
  guidance_internal = heartbeat->guidance_internal;
  enable_acs = heartbeat->enable_acs;
  enable_tvc = heartbeat->enable_tvc;
  enable_engine = heartbeat->enable_engine;
  safe_engine = heartbeat->safe_engine;
  power_engine = heartbeat->power_engine;
  engine_thrust = heartbeat->engine_thrust;
  engine_hover = heartbeat->engine_hover_height;
  tvc_angle1 = heartbeat->tvc_angle1;
  tvc_angle2 = heartbeat->tvc_angle2;
  last_heartbeat = std::chrono::system_clock::now();
  imu_calibration_status = heartbeat->imu_calibration_status;
  // add TVC calibration
  // add GPS calibration
}

void emplace_buffer(uint8_t* buf, size_t* offset, void* data, size_t sz) {
  memcpy(buf + (*offset), data, sz);
  *offset += sz;
}

void access_buffer(uint8_t* buf, size_t* offset, void* data, size_t sz) {
  memcpy(data, buf + (*offset), sz);
  *offset += sz;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STM32Bridge>(std::string(argv[1]), atoi(argv[2])));
  rclcpp::shutdown();
  return 0;
}
