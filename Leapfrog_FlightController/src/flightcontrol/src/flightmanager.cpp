/*===================================================================
 * Project    : LEAPFROG
 * File       : flightmanager.cpp
 * Author     : Antariksh Narain (2020) | Updated by BillyChrist (2025)
 * Description: Core runtime logic for LEAPFROG FlightManager ROS 2 node.
 *
 * Responsibilities:
 *  - Interfaces with onboard flight subsystems via serial and ROS 2
 *  - Runs the SerialMonitor loop, handling input from groundstation
 *  - Responds to commands for engine, ACS, sensors, gimbal, and scripts
 *  - Publishes heartbeat messages with real-time system status
 *  - Loads and executes onboard scripts defined in custom_scripts.list
 *  - Manages system shutdown safety procedures on communication timeout
 *
 * Execution:
 *  This node is launched manually or via ROS 2 launch file.
 *  It requires two command-line arguments:
 *      1. Serial port path (e.g., /dev/ttyAMA0)
 *      2. Baud rate (e.g., 9600)
 *
 * Logging:
 *  Uses rclcpp::get_logger("rcltelemcpp") for structured output
 *
 *===================================================================*/


#include "flightmanager.hpp"
#include "Communication.hpp"
#include "heartbeat.pb.h"

typedef std::chrono::steady_clock stClock;

FlightManager::FlightManager(string port, int baudrate, std::future<void> fut) : Node("FlightManager"), comm_(std::make_unique<Utilities::Communication>(port, baudrate)) {
    // Load Script files
    string filename = this->path_to_files + this->custom_script_file;
    ifstream file;
    file.open(filename.c_str(), ios::in);
    string data;
    string script_names = "";
    int num = 1;
    if (file.is_open()) {
        while (getline(file, data)) {
            this->script_map.push_back(this->path_to_files + data);
            script_names += "| " + to_string(num++) + " -> " + this->path_to_files + data + " |\n";
        }
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Script file not loaded.");
    }

    this->InitializeSequence();
    thread(&FlightManager::SerialMonitor, this, move(fut)).detach();
    // Send ready message
    string script_msg = (script_names == "") ? "No Loaded Scripts" : "Loaded Scripts (" + to_string(this->script_map.size()) + "):\n" + script_names;
    string temp_msg = "Welcome to LEAPFROG: Vehicle is Ready!\n" + script_msg;
    comm_->SendSerial(temp_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: %s", getTimeSinceEpoch(), temp_msg.c_str());
}

float FlightManager::getTimeSinceEpoch() {
    stClock::time_point t1 = stClock::now();
    std::chrono::duration<float> time_span = std::chrono::duration_cast<std::chrono::duration<float>>(t1 - tEpoch);
    return time_span.count();
}

void FlightManager::SerialMonitor(std::future<void> fut) {
	// ROS2 logging system: RCLCPP_INFO(logger, "format_string", args...);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Started Serial Monitor.", getTimeSinceEpoch());
    
    // heartbeat is handled by ROS 2 timer
    uint8_t loop_sleep_time = 50; // time in milliseconds
    std::vector<uint8_t> data_bytes;
    std::string data;
    
	while (fut.wait_for(chrono::milliseconds(loop_sleep_time)) == std::future_status::timeout) {
		if (this->IsAvailable()) {
            // Receive raw bytes from groundstation
            data_bytes = comm_->RecvSerialRaw();
            leapfrog::Command cmd_msg;
            if (cmd_msg.ParseFromArray(data_bytes.data(), data_bytes.size())) {
                std::string cmd = cmd_msg.command_text();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Protobuf Command Received: %s", getTimeSinceEpoch(), cmd.c_str());
                if (cmd == "exit") { break; }
                std::string response = this->Parser(cmd);
                // Optionally send a response as a protobuf message (not required for heartbeat flow)
            } else {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "t=%.4fs: Failed to parse protobuf Command message from groundstation.", getTimeSinceEpoch());
            }
		}
		// No heartbeat counter logic - heartbeat is handled by ROS 2 timer in InitializeSequence()
	}
	this->ShutdownSequence();
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Stopped Serial Monitor.", getTimeSinceEpoch());
}

void FlightManager::InitializeSequence() {
	// Initialize client services

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Initialized Clients", getTimeSinceEpoch());

	this->autorun = this->create_wall_timer(100ms, [this]() -> void {
		flightcontrol::msg::Heartbeat heartbeat;
		if (std::chrono::system_clock::now() - lastPacketTime > groundstation_timeout) {
			// Timeout
			// We could just not publish anything and let the cascaded heartbeats take care of it, but if we want to hit a safe state ASAP
			// it's faster and better to actively say "turn off"
			heartbeat.guidance_internal = false;
			heartbeat.enable_acs = false;
			heartbeat.enable_engine = false;
			heartbeat.safe_engine = false;
			heartbeat.power_engine = false;
			heartbeat.enable_tvc = false;
		}
		else {
			heartbeat.guidance_internal = guidance_internal;
			heartbeat.enable_acs = enable_acs;
			heartbeat.enable_engine = enable_engine;
			heartbeat.safe_engine = safe_engine;
			heartbeat.power_engine = power_engine;
			heartbeat.enable_tvc = enable_tvc;
		}
		// These have no effect if systems are disabled, so send them anyway for simplicity and filtering (if necessary)
		heartbeat.tvc_angle1 = tvc_angle1;
		heartbeat.tvc_angle2 = tvc_angle2;
		heartbeat.engine_hover_height = engine_height;
		heartbeat.engine_thrust = engine_thrust_cmd;

        // Handle IMU calibration timeout
        if (imu_calibrate_flag) {
            imu_calibrate_counter++;
            if (imu_calibrate_counter >= 150) {  // 15 seconds at 100ms heartbeat rate
                imu_calibrate_flag = false;
                imu_calibrate_counter = 0;
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "t=%.4fs: IMU calibration timeout reached", getTimeSinceEpoch());
            }
        }
        heartbeat.imu_calibration_status = imu_calibrate_flag;

        // Handle TVC calibration timeout
        if (tvc_calibrate_flag) {
            tvc_calibrate_counter++;
            if (tvc_calibrate_counter >= 150) {  // 15 seconds at 100ms heartbeat rate
                tvc_calibrate_flag = false;
                tvc_calibrate_counter = 0;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: TVC calibration timeout reached", getTimeSinceEpoch());
            }
        }
        heartbeat.tvc_calibration = tvc_calibrate_flag;

        // Publish heartbeat
		heartbeat_publisher_->publish(heartbeat);
		this->SendProtobufHeartbeat();
	});

	this->heartbeat_publisher_ = this->create_publisher<flightcontrol::msg::Heartbeat>("heartbeat", 1);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Initialized Subscriptions.", getTimeSinceEpoch());
}

void FlightManager::ShutdownSequence() {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Shutdown Sequence: Started", getTimeSinceEpoch());
	// Power off engine
	this->enable_engine = false;     // Disable engine
	this->enable_tvc = false;     // Disable TVC
	this->tvc_enable(0);
}

string FlightManager::label_run(string& name) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Labeling name: %s", getTimeSinceEpoch(), name.c_str());
    return "OK";
}

void FlightManager::ScriptRunner(string filename, future<void> script_future) {
    // load commands to memory
    ifstream file;
    file.open(filename.c_str(), ios::in);
    vector<string> commands;
    vector<int> cmd_delays;
    string data;
    if (file.is_open()) {
        bool toggle = true;
        while (getline(file, data))
        {
            if (toggle) {
                commands.push_back(data);
            }
            else {
                cmd_delays.push_back(atoi(data.c_str()));
            }
            toggle = !toggle;
        }
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Custom script %s does not exist.", getTimeSinceEpoch(), filename.c_str());
        return;
    }
    if (commands.size() != cmd_delays.size() || commands.size() == 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Script is not valid. Commands: %zu -> Delays: %zu.", getTimeSinceEpoch(), commands.size(), cmd_delays.size());
        return;
    }
    string notify = "Script Started: " + filename;
    comm_->SendSerial(notify);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Started custom script %s.", getTimeSinceEpoch(), filename.c_str());
    int i = 0;
    do {
        string resp = this->Parser(commands[i]);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Executed %s with response: %s.", getTimeSinceEpoch(), commands[i].c_str(), resp.c_str());
        comm_->SendSerial(resp);
        //std::this_thread.sleep_for(std::chrono::milliseconds(cmd_delays[i]));
        i++;
    } while (i < (int)commands.size() && script_future.wait_for(chrono::milliseconds(cmd_delays[i])) == std::future_status::timeout);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "t=%.4fs: Completed custom script %s. %zu / %zu.", getTimeSinceEpoch(), filename.c_str(), static_cast<size_t>(i), commands.size());
	notify = "Script completed: " + filename;
    comm_->SendSerial(notify);
    this->enable_script = 0;
}

string FlightManager::cmd_script(int value) {
    // Command to interrupt script
    if (value == 0) {
        this->enable_script = 0;
        if (exit_script_thread_promise != NULL) {
            exit_script_thread_promise->set_value();
            exit_script_thread_promise = NULL;
            return "OK - Stopping Running Scripts.";
        }
        return "OK - No script running.";
    }
else {
    if (this->enable_script > 0) {
        return "Script already running.";
    }

    if (value <= 0 || value > static_cast<int>(this->script_map.size())) {
        return "Invalid Script number " + std::to_string(value);
    }

    // List all available script files
    for (const auto& file : this->script_map) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "t=%.4fs: Available Script: %s",
            getTimeSinceEpoch(),
            file.c_str()
        );
    }

    // Launch the script
    exit_script_thread_promise = new std::promise<void>();
    std::future<void> exit_script_thread_future = exit_script_thread_promise->get_future();

    std::string script_file = this->script_map[value - 1];
    std::thread(
        &FlightManager::ScriptRunner,
        this,
        script_file,
        std::move(exit_script_thread_future)
    ).detach();

    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "t=%.4fs: Starting script %s for %d.",
        getTimeSinceEpoch(),
        script_file.c_str(),
        value
    );

    this->enable_script = value;

    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "t=%.4fs: Command Script state updated to %d",
        getTimeSinceEpoch(),
        value
    );

    return "OK";
	}
}


string FlightManager::engine_ctrl(int value) {
	safe_engine = value > 0;
	RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine Safe state updated %d", getTimeSinceEpoch(), value);
	return "OK";
}

string FlightManager::engine_power(int value) {
	if (safe_engine) {
		power_engine = value > 0;
		RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine Power state updated %d", getTimeSinceEpoch(), value);
		return "OK";
	}
	else {
		return "Engine safety must be enabled (engine ctrl 1)";
	}
}

string FlightManager::engine_enable(int value) {
	if (safe_engine && power_engine) {
		enable_engine = value > 0;
		RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine state updated %d", getTimeSinceEpoch(), value);
		return "OK";
	}
	else {
		return "Engine safety must be enabled (engine ctrl 1) and ECU must be powered on (engine power 1)";
	}
}

string FlightManager::engine_telem_0() {
	RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine Telemetry requested", getTimeSinceEpoch());
	return this->sub_engine;
}

string FlightManager::engine_telem_1() {return "Deprecated: Fuel Pressure";}
string FlightManager::engine_telem_2() {return "Deprecated: Igniter status";}
string FlightManager::engine_telem_3() {return "Deprecated";}
string FlightManager::engine_telem_4() {return "Deprecated";}

string FlightManager::engine_thrust(float value) {
	if (enable_engine && !guidance_internal) {
		engine_thrust_cmd = value;
		RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine Thrust state updated %f", getTimeSinceEpoch(), value);
		return "OK";
	}
	else {
		return "Engine must be enabled and guidance must not be internal";
	}
}

string FlightManager::engine_thrust2(int /*value*/) {return "Deprecated";}

string FlightManager::engine_hover(float value) {
	if (enable_engine && guidance_internal) {
		engine_height = value;
		RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Engine Hover state updated %f", getTimeSinceEpoch(), value);
		return "OK";
	}
	else {
		return "Engine must be enabled and guidance must be internal";
	}
}

string FlightManager::acs_enable(int value) {
	enable_acs = value > 0;
	RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: ACS state updated %d", getTimeSinceEpoch(), value);
	return "OK";
}

string FlightManager::imu_calibrate() {
    imu_calibration_flag = true;
    imu_calibration_counter = 0;
    RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Recalibrating IMU", getTimeSinceEpoch());
    return "Recalibrating IMU...";
}

string FlightManager::cmd_echo(int value) {return "Deprecated";}

string FlightManager::tvc_enable(int value) {
    enable_tvc = value > 0;
    RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: TVC state updated %d", getTimeSinceEpoch(), value);
    return "OK";
}

string FlightManager::tvc_calibrate(int value) {
    tvc_calibrate_flag = value > 0;
    RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: TVC calibration requested", getTimeSinceEpoch());
    return "OK";
}

string FlightManager::tvc_move(float angles[2]) {
    tvc_angle1 = angles[0];
    tvc_angle2 = angles[1];
    RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: TVC moved %f, %f", getTimeSinceEpoch(), angles[0], angles[1]);
    return "OK";
}

void FlightManager::feed_watchdog() {
	lastPacketTime = std::chrono::system_clock::now();
}

string FlightManager::guidance_enable(int value) {
	guidance_internal = value > 0;
	RCLCPP_INFO(rclcpp::get_logger("rcltelemcpp"), "t=%.4fs: Guidance state updated %d", getTimeSinceEpoch(), value);
	return guidance_internal ? "Guidance is going internal!" : "Switched to manual control";
}

void FlightManager::SendProtobufHeartbeat() {
    leapfrog::Heartbeat hb_msg;
    // IMU Data (TODO: assign real values when available)
    // If you have these as member variables, use them. Otherwise, leave a single TODO.
    // Example: hb_msg.set_roll_deg(roll_deg);
    // For now, leave a single TODO for IMU fields:
    // TODO: Populate IMU fields from STM32 telemetry when available
    hb_msg.set_roll_deg(0.0f);
    hb_msg.set_pitch_deg(0.0f);
    hb_msg.set_yaw_deg(0.0f);
    hb_msg.set_acc_x_g(0.0f);
    hb_msg.set_acc_y_g(0.0f);
    hb_msg.set_acc_z_g(0.0f);
    hb_msg.set_angvel_x_degs(0.0f);
    hb_msg.set_angvel_y_degs(0.0f);
    hb_msg.set_angvel_z_degs(0.0f);

    // TVC Data
    hb_msg.set_tvc_a_pos(tvc_angle1);
    hb_msg.set_tvc_b_pos(tvc_angle2);

    // Engine Telemetry (TODO: assign real values when available)
    hb_msg.set_engine_turbine_rpm(0);
    hb_msg.set_engine_rpm_setpoint(0);
    hb_msg.set_engine_egt_c(0);
    hb_msg.set_engine_pump_voltage(0.0f);
    hb_msg.set_engine_turbine_state(0);
    hb_msg.set_engine_off_condition(0);
    hb_msg.set_engine_throttle_percent(0);
    hb_msg.set_engine_current_a(0.0f);

    // Altitude
    hb_msg.set_altitude(0);

    // System Status
    static uint32_t heartbeat_counter = 0;
    hb_msg.set_heartbeat_counter(heartbeat_counter++);
    hb_msg.set_guidance_internal(guidance_internal);
    hb_msg.set_enable_acs(enable_acs);
    hb_msg.set_enable_tvc(enable_tvc);
    hb_msg.set_enable_engine(enable_engine);
    hb_msg.set_imu_calibration_status(imu_calibration_flag);

    std::string out;
    hb_msg.SerializeToString(&out);
    std::vector<uint8_t> out_bytes(out.begin(), out.end());
    comm_->SendSerialRaw(out_bytes);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    if (argc < 3) {
        printf("Pass port and baudrate as parameters");
        return -1;
    }
    promise<void> exit_signal;
    future<void> exit_future = exit_signal.get_future();
    rclcpp::spin(std::make_shared<FlightManager>(std::string(argv[1]), atoi(argv[2]), move(exit_future)));
    exit_signal.set_value();
    printf("Flight Manager: Sleeping for 5 seconds for things to shut down.");
    this_thread::sleep_for(chrono::seconds(5));
    printf("LEAPFROG Shutting down.");
    rclcpp::shutdown();
    return 0;
}
