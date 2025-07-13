/*---------------------------------------------------------------
 * Copyright (c) 2025 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain | Maintained by BillyChrist (2025)
 * Description: Class definition for communication module for encoding and decoding data
----------------------------------------------------------------- */

#include "Communication.hpp"
#include <vector>

using namespace std;

//#define TEST_COMMUNICATION

namespace Utilities
{
Communication::Communication(string channel, int baud_rate)
    : Serial(channel, baud_rate, '\n', 1000, -1)        
// Serial(string port, int baud, char terminator, int timeout, int flags)
// Update timeout delay if timing out before packets arrive, increase to add safety margin.
// Timeout delay should match expected data packet delivery (currently 1000 ms)
{
    this->channel   = channel;
    this->baud_rate = baud_rate;
    printf("Init Comm on %s at %d\n", this->channel.c_str(), this->baud_rate);
}
    void Communication::SendSerial(string data)
    {
        if(this->Send(this->convert_to_bytes(data)))
        {
            printf("%s Send: %s\n", this->channel.c_str(), data.c_str());
        }
    }
    string Communication::RecvSerial()
    {
       return string(this->Recv().begin(), this->Recv().end());
    }
    void Communication::SendSerialRaw(const std::vector<uint8_t>& data) {
        this->Send(data, true); // true: send delimiter if needed
    }
    std::vector<uint8_t> Communication::RecvSerialRaw() {
        return this->Recv();
    }
}

#ifdef TEST_COMMUNICATION
#include <iostream>
#include <unistd.h>

int main(int argv, char *argc[])
{
    if(argv != 2)
    {
        cout<< "Pass port name";
        return 1;
    }
    Utilities::Communication comm(argc[1], B9600);
    comm.SendSerial("Test1 Send String from " + string(argc[1]));
    comm.SendSerial("\4Test2 Sending String from " + string(argc[1]));
    sleep(2);
    string recv = comm.RecvSerial();
    while(recv.length() == 0)
    {
        printf("Waiting %s\n", recv.c_str());
        recv = comm.RecvSerial();
    }
    cout << argc[1] << " Received: " << recv << endl;
    return 0;
}
#endif