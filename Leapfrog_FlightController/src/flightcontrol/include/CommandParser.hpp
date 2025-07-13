/*===================================================================
 * Project    : LEAPFROG
 * File       : CommandParser.hpp
 * Author     : Antariksh Narain (2020) | Maintained by BillyChrist (2025)
 * Description: Abstract interface for parsing incoming command strings.
 * 
 * This class provides:
 *  - Parser() dispatcher that routes commands to subsystem-specific handlers
 *  - Helper methods for command tokenization and mapping
 * 
 * All subsystem handlers (e.g., engine_ctrl, acs_fire, etc.) are defined
 * as virtual functions to be implemented by inheriting classes.
 *
 * Usage:
 *  Inherit from CommandParser and override all required subsystem methods.
 *===================================================================*/


#ifndef _CMDPARSER_h_
#define _CMDPARSER_h_

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <cstdint>

using namespace std;

class CommandParser
{
private:
    const string INVALID_COMMAND = "Invalid Command.";
    map<string, int> cg_map = {{"r", 0}, {"R", 1}, {"p", 2}, {"P", 3}, {"y", 4}, {"Y", 5}};

    // \brief Split string by the delimiter
    // \param string data
    // \param character delimiter
    // \return vector of split strings
    vector<string> split(string data, char delim);

protected:
    // \brief Command Parser for Name
    // \param Name to label run
    // \return status
    string labelParser(string name);

    // \brief Command Parser for Engine
    // \param command to execute
    // \param additional params
    // \return status
    string engineParser(string cmd, string values);

    // \brief Command Parser for IMU
    // \param command to execute
    // \param additional params
    // \return status
    string imuParser(string cmd, string values);

    // \brief Command Parser for Miscellaneous commands
    // \param command to execute
    // \param additional params
    // \return status
    string cmdParser(string cmd, string values);

    // \brief Command Parser for TVC
    // \param command to execute
    // \param additional params
    // \return status
    string tvcParser(string cmd, string values);

    // \brief Command Parser for Guidance
    // \param command to execute
    // \param additional params
    // \return status
    string guidanceParser(string cmd, string values);
    

    
public:
    // \brief Primary Command Parser.
    // \param command to execute
    // \return status
    string Parser(string cmd);

    virtual string label_run(string name)
    {
        return "Labeling name to " + name;
    }
    virtual string engine_ctrl(int value)
    {
        return value == 1 ? "Control enabled" : "Control disabled";
    }
    virtual string engine_power(int value)
    {
        return value == 1 ? "Power enabled" : "Power disabled";
    }
    virtual string engine_enable(int value)
    {
        return value == 1 ? "Engine started" : "Engine stopped";
    }
    virtual string engine_telem_0()
    {
        return "Telemetry of Health Check";
    }
    virtual string engine_telem_1()
    {
        return "Telemetry of System Info";
    }
    virtual string engine_telem_2()
    {
        return "Engine Telemetry";
    }
    virtual string engine_telem_3()
    {
        return "Fuel Telemetry";
    }
    virtual string engine_telem_4()
    {
        return "Engine System status";
    }
    virtual string engine_thrust(float value)
    {
        return "Setting thrust to " + to_string(value);
    }
    virtual string engine_thrust2(int value)
    {
        return "Setting thrust2 to " + to_string(value);
    }
    virtual string engine_hover(float height) {
        return "Setting hover height to " + to_string(height);
    }

    virtual string acs_enable(int value)
    {
        return value == 1 ? "acs enabled" : "acs disabled";
    }
    virtual string acs_fire(int durations[6])
    {
        char temp[] = {'r', 'R', 'p', 'P', 'y', 'Y'};
        string send_string = "";
        for (int i = 0; i < 6; i++)
        {
            send_string += "Activating " + to_string(temp[i]) + " for " + to_string(durations[i]) + '\n';
        }
        return send_string;
    }
    virtual string imu_calibration()
    {
        return "Recalibrating IMU";
    }

    virtual string cmd_echo(int value)
    {
        return value == 1 ? "Echo enabled" : "Echo disabled";
    }
    virtual string cmd_script(int value)
    {
        return value == 1 ? "Script enabled" : "Script disabled";
    }

    virtual string tvc_enable(int value)
    {
        return value == 1 ? "TVC enabled" : "TVC disabled";
    }
    virtual string tvc_calibrate(int value)
    {
        return "TVC calibration for the" + to_string(value);
    }
    virtual string tvc_move(float angles[2])
    {
        char temp[] = {'r', 'p'};
        string send_string = "";
        for (int i = 0; i < 2; i++)
        {
            send_string += "Activating " + to_string(temp[i]) + " for " + to_string(angles[i]) + '\n';
        }
        return send_string;
    }
    virtual void feed_watchdog() {};
    virtual string guidance_enable(int value) { return value == 1 ? "guidance enabled" : "guidance disabled"; };
};

#endif
