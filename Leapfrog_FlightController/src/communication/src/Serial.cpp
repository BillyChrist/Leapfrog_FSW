/*==================================================================================
 * File        : Serial.cpp
 * Author      : Antariksh Narain (2020) | Modified by: BillyChrist (2025)
 * Project     : LEAPFROG
 * Description : Implements the Serial class for handling low-level serial communication.
 *               This includes port setup, read/write handling, packet buffering,
 *               and queue management using a background thread.
 *
 * Key Features:
 * - Configurable baud rate, termination character, and packet mode
 * - Supports both delimiter-based and fixed-size packet modes
 * - Threaded background reader queues complete packets for non-blocking access
 * - Optional logging and test modes (see TEST_SERIAL and TEST_SERIAL2)
 *
 * Usage Notes:
 * - Uses Linux POSIX APIs (termios) — intended for Raspberry Pi or Linux systems
 * - Timeout is managed via termios VTIME (default = 1s)
 * - Incoming data is stored in a circular queue up to `queue_size`
 *
 * License     : MIT
 *==================================================================================*/


#include "Serial.hpp"
#include <sys/ioctl.h>

Serial::~Serial()
{
    this->exit_signal.set_value();
    printf("Stopping Serial Communication %s.\n", this->port_name.c_str());
    sleep(1);
    close(this->serial_port);
}

Serial::Serial(string portname, int baudrate, char delim = '\n', int q_size = 100, int pkt_size = -1)
{
    this->port_name = portname;
    this->baud_rate = baudrate;
    this->delimiter = delim;
    this->queue_size = q_size;
    this->num_bytes = pkt_size;
    this->exit_future = this->exit_signal.get_future();
    if (!this->initializePort())
    {
        return;
    }
    // Initialize thread to monitor port
    thread(&Serial::manageQueue, this, std::move(this->exit_future)).detach();
    printf("Started Serial communication at %s.\n", this->port_name.c_str());
}

bool Serial::initializePort()
{
    this->serial_port = open(this->port_name.c_str(), O_RDWR);

    if (tcgetattr(this->serial_port, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return false;
    }
    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag &= ~(PARODD | CMSPAR);

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHOK;                                                       
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~(IEXTEN | ECHOCTL | ECHOKE);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    tty.c_iflag &= ~(IUCLC | INPCK);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_oflag &= ~OCRNL;
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    speed_t baud = B115200;
    switch(this->baud_rate) {
        case 9600:
            baud = B9600; break;
        case 57600:
            baud = B57600; break;
        case 115200:
            baud = B115200; break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return false;
    }
    tcflush(this->serial_port,TCIOFLUSH);

    return true;
}

void Serial::manageQueue(std::future<void> _future)
{
    uint8_t c;
    vector<uint8_t> data;
    //memset(c, '\0', sizeof(c));
    int size = read(this->serial_port, &c, 1);
    printf("Starting Serial Queue Manager for %s.\n", this->port_name.c_str());
    while (1)
    {
        if (size <= 0)
        {
            // buffer is empty sleep for a while
            if (_future.wait_for(chrono::milliseconds(RATE)) != std::future_status::timeout)
            {
                break;
            }
#ifdef DEBUG_SERIAL
            printf("Sleeping for a while!\n");
#endif
        }
        //else if ((this->num_bytes == -1 && c == this->delimiter) || (this->num_bytes == (int)data.size()))
        else if (this->num_bytes == -1 && c == this->delimiter)
        {
            this->_mutex.lock();
            this->recv_data.push(data);
            if (this->recv_data.size() > this->queue_size)
            {
                this->recv_data.pop();
            }
            this->_mutex.unlock();
            data.clear();
        }
        else if (this->num_bytes == (int)data.size())
        {
#ifdef DEBUG_SERIAL
            //printf("Pushing---");
            for (int i = 0; i < this->num_bytes; i++)
            {
                printf("%d,", data[i]);
            }
#endif
            this->_mutex.lock();
            this->recv_data.push(data);
            if (this->recv_data.size() > this->queue_size)
            {
                this->recv_data.pop();
            }
            this->_mutex.unlock();
            data.clear();
            data.push_back(c);
        }
        else
        {
            data.push_back(c);
        }
        size = read(this->serial_port, &c, 1);
    }
    printf("Closing Queue Manager for %s, end of transmission received.\n", this->port_name.c_str());
}

bool Serial::Send(vector<uint8_t> data, bool send_delim)
{
    if (send_delim) {
        data.push_back(this->delimiter);
    }
    //printf("Sending data of size %d \n", (int)data.size());
    if (write(this->serial_port, &data[0], data.size()) < 0)
    {
        return false;
    }
    return true;
}

bool Serial::Send(string data)
{
    return this->Send(this->convert_to_bytes(data));
}

vector<uint8_t> Serial::Recv()
{
    vector<uint8_t> data;
    this->_mutex.lock();
    data = this->recv_data.front();
    this->recv_data.pop();
    this->_mutex.unlock();
    return data;
}

string Serial::convert_to_string(vector<uint8_t> data)
{
    string str = "";
    for (const uint8_t b : data)
    {
        str += b;
    }
    return str;
}

vector<uint8_t> Serial::convert_to_bytes(string str)
{
    vector<uint8_t> data;
    for (int i = 0; i < (int)str.size(); i++)
    {
        data.push_back(str[i]);
    }
    return data;
}

int Serial::IsAvailable()
{
    int available_size;
    this->_mutex.lock();
    //available_size = this->recv_data.size() > 0;
    available_size = this->recv_data.size();
    this->_mutex.unlock();
    return available_size;
}

#ifdef TEST_SERIAL
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        cout << "Pass port name and baud rate.\n";
        return 1;
    }
    Serial serial(argv[1], atoi(argv[2]), '\n');
    serial.Send(serial.convert_to_bytes("Test1 Send String from " + string(argv[1])));
    sleep(2);
    while (1)
    {
        if (serial.IsAvailable())
        {
            string recv = serial.convert_to_string(serial.Recv());
            cout << argv[1] << " Received: " << recv << endl;
            break;
        }
        sleep(1);
    }
    printf("Exiting main thread!\n");
    return 0;
}
#endif

#ifdef TEST_SERIAL2
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        cout << "Pass port name and baud rate.\n";
        return 1;
    }
    Serial serial(argv[1], atoi(argv[2]), 0, 1, 9);
    serial.Send(argv[1]);
    sleep(2);
    while (1)
    {
        if (serial.IsAvailable())
        {
            string recv = serial.convert_to_string(serial.Recv());
            cout << argv[1] << " Received: " << recv << endl;
            break;
        }
        sleep(1);
    }
    printf("Exiting main thread!\n");
    return 0;
}
#endif
