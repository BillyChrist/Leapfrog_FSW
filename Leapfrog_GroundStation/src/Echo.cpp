/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main program for Ground station
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

// STL Libs
#include <iostream>
#include <functional>
#include <stdio.h>
#include <string>
#include <string.h>
#include <chrono>
#include <thread>
#include <future>
#include <map>
#include <mutex>
#include <random>
#include <chrono>
// User Defined Libs
#include <Serial.hpp>
// Add protobuf includes
#include "heartbeat.pb.h"

using namespace std;

Serial *serial;
mutex _mutex;

// command, duration_ms
#define TOTAL_CMDS 2 // 1 + num in heart_beats
map<string,int> heart_beats = 
{
    {"t 1",10000}
};

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
Clock::time_point t0;
Clock::time_point t1;
bool echo_flag = true;

// Add a DEBUG flag
#define DEBUG true

void Receiver(std::future<void> fut)
{
    printf("\rStarted Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(100)) == std::future_status::timeout)
    {
        if(serial->IsAvailable())
        {
            auto recv = serial->Recv();
            leapfrog::Heartbeat heartbeat;
            if (heartbeat.ParseFromArray(recv.data(), recv.size())) {
                if (DEBUG) {
                    printf("[HEARTBEAT] roll: %.2f, pitch: %.2f, yaw: %.2f, altitude: %d\n",
                        heartbeat.roll_deg(), heartbeat.pitch_deg(), heartbeat.yaw_deg(), heartbeat.altitude());
                    // Print more fields as needed
                }
            } else {
                if (DEBUG) printf("[WARN] Failed to parse Heartbeat protobuf message.\n");
            }
        }
    }
    printf("Exiting Receiver!\n");
}

string random_string(std::size_t length)
{
    const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);

    std::string random_string;

    for (std::size_t i = 0; i < length; ++i)
    {
        random_string += CHARACTERS[distribution(generator)];
    }

    return random_string;
}

void Sender(std::promise<void> prom)
{
    printf("Started Sender...\n");
    char data[256];
    while (1)
    {
        cin.getline(data, sizeof(data));
        if (string(data) == "exit")
        {
            printf("Exiting ...\n");
            leapfrog::Command cmd;
            cmd.set_command_text("exit");
            std::string out;
            cmd.SerializeToString(&out);
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
            prom.set_value();
            break;
        }
        else
        {
            leapfrog::Command cmd;
            cmd.set_command_text(data);
            std::string out;
            cmd.SerializeToString(&out);
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
        }
    }
    printf("Exiting Sender!\n");
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }

    serial = new Serial(argv[1], atoi(argv[2]),'\n',100,-1);

    // Promise to manage threads
    promise<void> exit_signal;
    future<void> exit_future = exit_signal.get_future();
    
    thread t2(&Receiver, move(exit_future));
    thread t1(&Sender, move(exit_signal));
    sleep(1);
    t1.join();
    t2.join();
    printf("Closed primary threads!\n");
    return 0;
}