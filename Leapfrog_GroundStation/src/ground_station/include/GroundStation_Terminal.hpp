/*---------------------------------------------------------------
 * Copyright (c) 2025 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : BillyChrist
 * Description: Header file for Ground Station Terminal
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

#ifndef GROUND_STATION_TERMINAL_HPP
#define GROUND_STATION_TERMINAL_HPP

#include <iostream>
#include <functional>
#include <string>
#include <chrono>
#include <thread>
#include <future>
#include <map>
#include <mutex>
#include <random>
#include <cstdio>
#include "Serial.hpp"

namespace leapfrog {

class GroundStationTerminal {
public:
    GroundStationTerminal();
    ~GroundStationTerminal();

    // Core functionality
    void start(const char* port_name, int baud_rate);
    void stop();

private:
    // Member variables
    Serial* serial_;
    std::mutex mutex_;
    int enable_echo_;
    bool tshb_;  // TroubleShooting HeartBeats
    int tshb_counter_;
    std::chrono::high_resolution_clock::time_point t0_;
    std::chrono::high_resolution_clock::time_point t1_;
    const std::chrono::steady_clock::time_point t_epoch_;

    // Helper functions
    float get_time_since_epoch();
    int get_int_from_data(const char data[], int index);
    std::string random_string(std::size_t length);
    void receiver(std::future<void> fut);
    void sender(std::promise<void> exit_promise, std::promise<void> exit_heartbeat_promise);
    void heartbeats(std::future<void> fut, std::string cmd, int sleep_time);

    // Thread management
    std::thread receiver_thread_;
    std::thread sender_thread_;
    std::thread heartbeat_thread_;
};

} // namespace leapfrog

#endif // GROUND_STATION_TERMINAL_HPP
