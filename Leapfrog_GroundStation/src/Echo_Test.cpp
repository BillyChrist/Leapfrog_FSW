/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main program for Ground station
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

// Include Libraries
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
#include <thread>
// User Defined Libs
#include <Serial.hpp>
// Add protobuf includes
#include "heartbeat.pb.h"

// Add a DEBUG flag
#define DEBUG true

using namespace std;

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }
    Serial serial(argv[1], atoi(argv[2]),'\n',1000,-1);
    printf("\rStarted Receiver...\n");
    while (1)
    {
        if(serial.IsAvailable())
        {
            auto recv = serial.Recv();
            leapfrog::Heartbeat heartbeat;
            if (heartbeat.ParseFromArray(recv.data(), recv.size())) {
                if (DEBUG) {
                    printf("[HEARTBEAT] roll: %.2f, pitch: %.2f, yaw: %.2f, altitude: %d\n",
                        heartbeat.roll_deg(), heartbeat.pitch_deg(), heartbeat.yaw_deg(), heartbeat.altitude());
                }
            } else {
                if (DEBUG) printf("[WARN] Failed to parse Heartbeat protobuf message.\n");
            }
            // Optionally echo back a command
            // leapfrog::Command cmd;
            // cmd.set_command_text("echo");
            // std::string out;
            // cmd.SerializeToString(&out);
            // serial.Send(vector<uint8_t>(out.begin(), out.end()), false);
        }
        else
        {
            this_thread::sleep_for(chrono::milliseconds(200));
            printf("Sleeping\n");
        }
    }
    printf("Exiting Receiver!\n");
    printf("Closed primary threads!\n");
    return 0;
}