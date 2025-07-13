/*---------------------------------------------------------------
 * Copyright (c) 2025 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : BillyChrist
 * Description: Header file for Ground Station
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

#ifndef GROUND_STATION_HPP
#define GROUND_STATION_HPP

#include <iostream>
#include <functional>
#include <string>
#include <chrono>
#include <thread>
#include <future>
#include <map>
#include <mutex>
#include <fstream>
#include <vector>
#include <cstdio>
#include "Serial.hpp"

namespace leapfrog {

class GroundStation {
public:
    GroundStation();
    ~GroundStation();

    // Core functionality
    void start(const char* port_name, int baud_rate, const char* monitoring_script, const char* test_script);
    void stop();

private:
    // Member variables
    Serial* serial_;
    std::mutex mutex_;
    int enable_echo_;
    std::chrono::high_resolution_clock::time_point t0_;
    std::chrono::high_resolution_clock::time_point t1_;

    // Helper functions
    std::string random_string(std::size_t length);
    void receiver(std::future<void> fut);
    void sender(std::vector<std::promise<void>> exit_promises, std::string filename);
    void heartbeats(std::future<void> fut, std::string cmd, int sleep_time);

    // Monitoring
    struct monitor {
        std::string command;
        int duration;
    };
    std::vector<monitor> load_monitoring(std::string filename);

    // Thread management
    std::vector<std::thread> monitoring_threads_;
    std::thread receiver_thread_;
    std::thread sender_thread_;
};

} // namespace leapfrog

#endif // GROUND_STATION_HPP
