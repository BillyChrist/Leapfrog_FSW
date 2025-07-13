/*===================================================================
 * Project    : LEAPFROG
 * File       : CommandParser.cpp
 * Author     : Antariksh Narain (2020) | Maintained by BillyChrist (2025)
 * Description: Implements the CommandParser class responsible for
 *              interpreting and dispatching incoming string commands.
 *
 * Each command string is expected in the format:
 *      "<subsystem> <command> <value>"
 * For example:
 *      "engine ctrl 1"          → calls engine_ctrl(1)
 *      "imu calibrate"          → triggers IMU calibration
 *      "tvc move 0.1,0.2"       → moves TVC to specified angles
 *
 * Supported subsystems:
 *      - engine
 *      - imu
 *      - cmd
 *      - tvc
 *      - guidance
 * 
 * Parser maps inputs to virtual functions which are expected to be
 * overridden by the owning node (e.g., FlightManager).
 *===================================================================*/
// TODO Update actuall commands (i.e.: "ACS systemEnabled")

#include "CommandParser.hpp"


/* Parsing Functions  ------------------------------------------------------- */


vector<string> CommandParser::split(string data, char delim) {
    vector<string> tokens;
    stringstream check(data);
    string temp;
    while (getline(check, temp, delim)) {
        tokens.push_back(temp);
    }
    return tokens;
}

string CommandParser::labelParser(string name) {
    return this->label_run(name);
}

string CommandParser::engineParser(string cmd, string values) {
    float value = atof(values.c_str());

    if (cmd == "ctrl") {
        // service
        return this->engine_ctrl(value);
    }
    else if (cmd == "power") {
        // service
        return this->engine_power(value);
    }
    else if (cmd == "enable") {
        // service
        return this->engine_enable(value);
    }
    else if (cmd == "telem") {
        if (value == 0) {
            // service x2
            return this->engine_telem_0();
        }
        else if (value == 1) {
            // Parameters
            return this->engine_telem_1();
        }
        else if (value == 2) {
            // Publisher
            return this->engine_telem_2();
        }
        else if (value == 3) {
            // Publisher
            return this->engine_telem_3();
        }
        else if (value == 4) {
            // Publisher
            return this->engine_telem_4();
        }
        else {
            return INVALID_COMMAND;
        }
    }
    else if (cmd == "thrust") {
        // Action service
        return this->engine_thrust(value);
    }
    else if (cmd == "thrust2") {
        // Action service
        return this->engine_thrust2(value);
    }
    else if (cmd == "hover") {
        return this->engine_hover(value);
    }
    else {
        return INVALID_COMMAND;
    }
}

string CommandParser::imuParser(string cmd, string values) {
    if (cmd == "calibrate") {
        return this->imu_calibrate();
    }
    return this->INVALID_COMMAND;
}

string CommandParser::cmdParser(string cmd, string values) {
    if (cmd == "echo") {
        int value = stoi(values);
        return this->cmd_echo(value);
    }
    else if (cmd == "script") {
        int value = stoi(values);
        return this->cmd_script(value);
    }
    return this->INVALID_COMMAND;
}

string CommandParser::tvcParser(string cmd, string values) {
    if (cmd == "enable") {
        // software flag update
        int value = atoi(values.c_str());
        return this->tvc_enable(value);
    }
    else if (cmd == "move") {
        // service
        float angles[2] = {0.0};
        vector<string> tokens = this->split(values, ',');
        for (int i=0; i<(int)tokens.size(); i++) {
            angles[i] = atof(tokens[i].c_str());
        }
        // If successful activate the thrusters
        return this->tvc_move(angles);
    }
    else if (cmd == "calibrate") {
        return this->tvc_calibrate(1);  // Always send 1 to trigger calibration
    }
    else {
        return INVALID_COMMAND;
    }
}

string CommandParser::guidanceParser(string cmd, string values) {
    if (cmd == "enable") {
        // software flag update
        int value = atoi(values.c_str());
        return this->guidance_enable(value);
    }
    else {
        return INVALID_COMMAND;
    }
}

string CommandParser::Parser(string cmd) {
    vector<string> tokens = this->split(cmd, ' ');
    if (tokens.size() < 2) {
        return this->INVALID_COMMAND;
    }
    if (tokens[0] == "label") {
        return this->labelParser(tokens[1]);
    }
    else if (tokens[0] == "engine") {
        return this->engineParser(tokens[1], tokens[2]);
    }
    else if (tokens[0] == "imu") {
        return this->imuParser(tokens[1], tokens[2]);
    }
    else if (tokens[0] == "cmd") {
        return this->cmdParser(tokens[1], tokens[2]);
    }
    else if (tokens[0] == "tvc") {
        return this->tvcParser(tokens[1], tokens[2]);
    }
    else if (tokens[0] == "guidance") {
        return this->guidanceParser(tokens[1], tokens[2]);
    }
    return this->INVALID_COMMAND;
}

#ifdef TEST_CMDPARSER
#include <iostream>
int main() {
    char data[100];
    CommandParser cp;
    while (1) {
        cin.getline(data, sizeof(data));
        if (string(data) == "exit") {break;}
        cout << cp.Parser(data) << endl;
    }
}
#endif
