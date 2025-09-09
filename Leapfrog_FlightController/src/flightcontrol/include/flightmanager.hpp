/*===================================================================
 * Project    : LEAPFROG
 * File       : flightmanager.hpp
 * Author     : Antariksh Narain (2020) | Maintained by BillyChrist (2025)
 * Description: Core flight manager class for LEAPFROG flight software.
 *
 * This class combines:
 *  - ROS 2 Node functionality (subscriptions, publishers, timers)
 *  - Serial communication (via custom Serial class)
 *  - Command parsing and execution logic (via CommandParser)
 *
 * Responsibilities:
 *  - Monitor and process serial commands from groundstation
 *  - Manage and publish system heartbeat messages
 *  - Execute engine, ACS, gimbal, and guidance actions
 *  - Run and manage onboard scripts
 *  - Interface with telemetry streams via ROS 2 topics
 *
 * Note:
 *  If running without onboard sensors, sensor subscriptions can be disabled.
 *
 *===================================================================*/

// Project headers
#include "Serial.hpp"
#include "flightcontrol/msg/heartbeat.hpp"
#include "CommandParser.hpp"

// ROS headers
#include "rclcpp/rclcpp.hpp"

// Standard headers
#include <chrono>
#include <memory>
#include <cstdio>
#include <string>
#include <future>
#include <thread>
#include <fstream>
#include <cstdlib>
#include <mutex>

#define HEARTBEAT_DURATION 2500

using namespace std::chrono_literals;

typedef std::chrono::steady_clock stClock;

class FlightManager : public rclcpp::Node, public Serial, public CommandParser
{
private:
    // ROS variables
    rclcpp::Publisher<flightcontrol::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
    rclcpp::Subscription<flightcontrol::msg::Heartbeat>::SharedPtr telemetry_subscription_;
    rclcpp::TimerBase::SharedPtr autorun;
    
    // Latest telemetry data from STM32 bridge
    flightcontrol::msg::Heartbeat latest_telemetry_;
    std::mutex telemetry_mutex_;

    // Local variables
    bool enable_engine = false;
    bool safe_engine = false;
    bool power_engine = false;
    bool enable_acs = false;
    bool enable_echo = false;
    bool enable_tvc = false;
    bool guidance_internal = false;
    int enable_script = 0;

    uint8 imu_calibration_status = 0;
    bool imu_calibration_flag = false;
    int imu_calibration_counter = 0;
    
    bool tvc_calibrate_flag;
    int tvc_calibrate_counter = 0;

    const std::chrono::milliseconds groundstation_timeout{500};
    std::chrono::time_point<std::chrono::system_clock> lastPacketTime;

    float tvc_angle1 = 0, tvc_angle2 = 0;
    float engine_height = 0, engine_thrust_cmd = 0;

    string sub_engine;

    promise<void> *exit_script_thread_promise = NULL;
    future<void> *exit_script_thread_future = NULL;
    thread script_thread;

    const string path_to_files = "/usr/local/share/script/";
    const string custom_script_file = "custom_scripts.list";
    vector<string> script_map;


public:
    FlightManager(string, int, std::future<void>);

    const stClock::time_point tEpoch = stClock::now();

    // \brief returns the time elapsed since the epoch time t0
    // \return number of seconds since epoch time t0
    float getTimeSinceEpoch();

    void SerialMonitor(std::future<void>);

    void InitializeSequence();

    void ShutdownSequence();

    void ScriptRunner(string filename, std::future<void>);

    virtual string label_run(string& name);

    virtual string engine_ctrl(int value);
    virtual string engine_power(int value);
    virtual string engine_enable(int value);
    virtual string engine_telem_0();
    virtual string engine_telem_1();
    virtual string engine_telem_2();
    virtual string engine_telem_3();
    virtual string engine_telem_4();
    virtual string engine_thrust(float value);
    virtual string engine_thrust2(int value);
    virtual string engine_hover(float height);

    virtual string acs_enable(int value);
    virtual string imu_calibration();

    virtual string cmd_echo(int value);
    virtual string cmd_script(int value);

    virtual string tvc_enable(int value);
    virtual string tvc_calibrate(int value);
    virtual string tvc_move(float angles[2]);

    virtual string guidance_enable(int value) override;

    virtual void feed_watchdog() override;

    // Heartbeat methods
    void SendProtobufHeartbeat();
    void telemetry_callback(const flightcontrol::msg::Heartbeat::SharedPtr msg);
};
