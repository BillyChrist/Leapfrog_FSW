/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: RF Communication Library for encoding and decoding data
----------------------------------------------------------------- */

#include "Serial.hpp"
#include <string>
#include <sstream>
#include <vector>
using namespace std;

namespace Utilities
{
    class Communication : private Serial
    {
    private:
        const int ROT13 = 13;
        const int ROT7 = 7;
        int baud_rate;
        string channel;

    protected:
        // (encoder/decoder removed)

    public:
        Communication() : Serial("/dev/ttyAMA0", 9600, '\n', 1000, -1) {}
        // \brief Initialize communication channel
        // \param channel the channel name e.g. /dev/pts/1
        // \param baud_rate the channel baud rate
        Communication(string, int);
        // \brief Send data on the Serial port
        // \param data to be sent
        void SendSerial(string);
        // \brief Receive data from Serial port
        // \return data string
        string RecvSerial();
        // \brief Send raw bytes on the Serial port (protobuf, for groundstation comms)
        // \param data: vector of bytes (protobuf-serialized message)
        void SendSerialRaw(const std::vector<uint8_t>& data);
        // \brief Receive raw bytes from Serial port (protobuf, for groundstation comms)
        // \return vector of bytes (protobuf-serialized message)
        std::vector<uint8_t> RecvSerialRaw();
    };
} // namespace Utilities