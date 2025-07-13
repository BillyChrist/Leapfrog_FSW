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
#include <cstdio>
#include <string>
#include <chrono>
#include <thread>
#include <future>
#include <map>
#include <mutex>
#include <random>
// User Defined Libs
#include "Serial.hpp"
#include "heartbeat.pb.h"
// #include <bits/stdc++.h>

using namespace std;

Serial *serial;
mutex _mutex;
int enable_echo = 0;
bool tshb = false; // TroubleShooting HeartBeats -> TSHB -> tshb
int tshb_counter = 0; // For when tshb gets turned on
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
typedef std::chrono::steady_clock stClock;
Clock::time_point t0;
Clock::time_point t1;

const stClock::time_point tEpoch = stClock::now();

// \brief returns the time elapsed since the epoch time t0
// \return number of seconds since epoch time t0
float getTimeSinceEpoch()
{
    stClock::time_point t1 = stClock::now();
    chrono::duration<float> time_span = chrono::duration_cast<chrono::duration<float>>(t1 - tEpoch);
    return time_span.count();
}

int getIntFromData(const char data[], int index)
{
    string data_str = string(data);
    int val_len = data_str.length()-index;
    char val[val_len+1]; // The +1 accounts for the fact that char arrays must end with a 0
    val[val_len] = 0;
    for (int i = 0; i < val_len; i++) {val[i] = data[index+i];}
    return atoi(val);
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

// Add a DEBUG flag
#define DEBUG true

void Receiver(std::future<void> fut)
{
    string recv;
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        if (serial->IsAvailable())
        {
            auto data = serial->Recv();
            leapfrog::Heartbeat heartbeat;
            if (heartbeat.ParseFromArray(data.data(), data.size())) {
                if (DEBUG) {
                    printf("[HEARTBEAT] roll: %.2f, pitch: %.2f, yaw: %.2f, altitude: %d\n",
                        heartbeat.roll_deg(), heartbeat.pitch_deg(), heartbeat.yaw_deg(), heartbeat.altitude());
                }
            } else {
                if (DEBUG) printf("[WARN] Failed to parse Heartbeat protobuf message.\n");
            }
        }
    }
    printf("Exiting Receiver!\n");
}

void Sender(promise<void> exit_promise, promise<void> exit_heartbeat_promise)
{
    char data[100];
    while (1)
    {
        cin.getline(data, sizeof(data));
        leapfrog::Command cmd;
        if (string(data) == "exit")
        {
            printf("t=%.4fs: ", getTimeSinceEpoch());
            printf("Exiting ...\n");
            cmd.set_command_text("exit");
            std::string out;
            cmd.SerializeToString(&out);
            _mutex.lock();
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
            _mutex.unlock();
            printf("t=%.4fs: ", getTimeSinceEpoch());
            printf("Waiting for 5 seconds for the vehicle to shutdown!\n");
            sleep(5);
            exit_promise.set_value();
            exit_heartbeat_promise.set_value();
            break;
        }
        else if (string(data) == "cmd echo 1")
        {
            enable_echo = 2;
            printf("t=%.4fs: ", getTimeSinceEpoch());
            printf("Echo Enabled!\n");
            cmd.set_command_text(data);
            std::string out;
            cmd.SerializeToString(&out);
            _mutex.lock();
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
            _mutex.unlock();
        }
        else if (string(data).rfind("tshb ") == 0)
        {
            // Keep legacy tshb logic if needed, or send as command
            cmd.set_command_text(data);
            std::string out;
            cmd.SerializeToString(&out);
            _mutex.lock();
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
            _mutex.unlock();
        }
        else
        {
            cmd.set_command_text(data);
            std::string out;
            cmd.SerializeToString(&out);
            _mutex.lock();
            serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
            _mutex.unlock();
            printf("t=%.4fs: ", getTimeSinceEpoch());
            printf("Sent %s\n", data);
        }
    }
    printf("Exiting Sender!\n");
}

void HeartBeats(std::future<void> fut, string cmd, int sleep_time)
{
    leapfrog::Command command;
    command.set_command_text(cmd);
    std::string out;
    unsigned long long int counter = 0;
    while (fut.wait_for(chrono::milliseconds(sleep_time)) == std::future_status::timeout)
    {
        _mutex.lock();
        serial->Send(vector<uint8_t>(out.begin(), out.end()), false);
        _mutex.unlock();
    }
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        //printf("Pass monitoring script name and test script name too.\n");
        return -1;
    }

    printf("Make sure fuel lines have been primed before firing engine!\n");
    
    printf("\nThe live-time telemetry file on the Pi is located at ~/vehicle/src/stm32_bridge/src/liveTelem.txt, and is written to from ~/vehicle/src/stm32_bridge/src/stm32_bridge.cpp\n");
    printf("The command to see the last telemetry packet (the contents of that file) in live time, once ssh’ed into the Pi is\n\twatch -n 1 cat ~/vehicle/src/stm32_bridge/src/liveTelem.txt\n");
    printf("The command to compile the live telemetry packet updates into a file on a local machine is\n\tssh ubuntu@192.168.0.100 'tail -f ~/vehicle/src/stm32_bridge/src/liveTelem.txt' >> fileDestinationOnLocalMachine\n\n");
    
    serial = new Serial(argv[1], atoi(argv[2]),'\n',1000,-1);
    //initialize();
    promise<void> exit_signal;
    promise<void> exit_heartbeat_signal;
    future<void> exit_future = exit_signal.get_future();
    future<void> heartbeat_exit_future = exit_heartbeat_signal.get_future();
    thread th_recv(&Receiver, move(exit_future));
    thread heartbeat(&HeartBeats, move(heartbeat_exit_future), "heartbeat", 100); // send command "heartbeat" every 100 ms
    printf("t=%.4fs: ", getTimeSinceEpoch());
    printf("Started Recevier Thread!\n");
    thread th_send(&Sender, move(exit_signal), move(exit_heartbeat_signal));
    printf("t=%.4fs: ", getTimeSinceEpoch());
    printf("Started Sender Thread!\n");

    th_send.join();
    th_recv.join();
    heartbeat.join();
    printf("t=%.4fs: ", getTimeSinceEpoch());
    printf("Closed primary threads! ");
    printf("Closed all threads safely!\n");
    
    printf("\nTo save the telemetry packets shown on the live terminal, type 'make download-live' into another terminal window.\n");
    printf("\nMake sure to purge and prime fuel lines before firing engine again!\n");
    return 0;
}
