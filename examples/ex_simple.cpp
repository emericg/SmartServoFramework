/*!
 * This file is part of SmartServoFramework.
 * Copyright (c) 2014, INRIA, All rights reserved.
 *
 * SmartServoFramework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/lgpl-3.0.txt>.
 *
 * \file ex_simple.cpp
 * \date 17/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

/*
 * Simple test program: initialize the "Simple API" with serial port
 * auto-detection, scan this serial port for servo(s), then let you control
 * 4 servos with the keypad:
 *
 * - Use "kp-middle" (or 5) to set all the servo at their initial positions (in
 * the middle of their step range).
 * - Use "kp-left" (4) and "kp-right" (6) to move servo 1.
 * - Use "kp-up" (8) and "kp-down" (2) to move servo 2.
 * - Use "kp-up/left" (7) and "kp-down/right" (3) to move servo 3.
 * - Use "kp-up/right" (9) and "kp-down/left" (1) to move servo 4.
 * - Use any other key to exit the program.
 */

// Smart Servo Framework
#include "../src/DynamixelSimpleAPI.h"

// C++ standard libraries
#include <iostream>
#include <map>

/* ************************************************************************** */

// ID of the servos you want to use in this test program:
#define ID_SERVO_1       1
#define ID_SERVO_2       2
#define ID_SERVO_3       3
#define ID_SERVO_4       4

/* ************************************************************************** */

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======== Smart Servo Framework Tester ========" << std::endl;

    // Initialize a Dynamixel "Simple API" instance
    DynamixelSimpleAPI dxl;

    // Initialize a serial link for the controller
    // You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
    // Note: serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.
    std::string deviceName = "auto";
    if (dxl.serialInitialize(deviceName, 1) == 0)
    {
        std::cerr << "> Failed to open a serial link for our SimpleAPI! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Scan serial port for servos, store results (device IDs found) inside a vector
    std::vector <int> ids = dxl.servoScan();
    std::map <int, int> mCurrentPos;
    std::map <int, int> mWantedPos;

    // Read initial position(s) for each servo found by the scan
    std::cout << std::endl << "Servos: [";
    for (std::vector <int>::iterator idsIterator = ids.begin(); idsIterator != ids.end(); idsIterator++)
    {
        // Read & store position
        int pos = dxl.readCurrentPosition(*idsIterator);
        mCurrentPos.insert(std::pair<int,int>(*idsIterator, pos));
        mWantedPos.insert(std::pair<int,int>(*idsIterator, pos));

        // Print id/position pair for current servo
        std::cout << "id: " << *idsIterator << " (pos:" << mCurrentPos.at(*idsIterator) << "), " ;
    }
    std::cout << "]" << std::endl;

    // Check we have the 4 servos we are supposed to have for this test program
    if (ids.size() < 3)
    {
        std::cerr << "We have " << ids.size() << " servos instead of the 4 needed for this test program! Exiting..." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << std::endl << "======== MAIN LOOP ========" << std::endl;

    std::cout << std::endl << "> Use the keypad to move (press ESC or any other key to exit)" << std::endl;
    system("/bin/stty raw");

    bool running = true;
    while (running)
    {
        // Read current position(s)
        for (std::vector <int>::iterator idsIterator = ids.begin(); idsIterator != ids.end(); idsIterator++)
        {
            mCurrentPos.at(*idsIterator) = dxl.readCurrentPosition(*idsIterator);
        }

        // Move order?
        int key = getchar();
        switch (key)
        {
        case 52: // kp 4
            mWantedPos.at(ID_SERVO_1) = mCurrentPos.at(ID_SERVO_1) - 32;
            break;
        case 54: // kp 6
            mWantedPos.at(ID_SERVO_1) = mCurrentPos.at(ID_SERVO_1) + 32;
            break;

        case 50: // kp 2
            mWantedPos.at(ID_SERVO_2) = mCurrentPos.at(ID_SERVO_2) - 32;
            break;
        case 56: // kp 8
            mWantedPos.at(ID_SERVO_2) = mCurrentPos.at(ID_SERVO_2) + 32;
            break;

        case 51: // kp 3
            mWantedPos.at(ID_SERVO_3) = mCurrentPos.at(ID_SERVO_3) - 32;
            break;
        case 55: // kp 7
            mWantedPos.at(ID_SERVO_3) = mCurrentPos.at(ID_SERVO_3) + 32;
            break;

        case 49: // kp 1
            mWantedPos.at(ID_SERVO_4) = mCurrentPos.at(ID_SERVO_4) - 32;
            break;
        case 57: // kp 9
            mWantedPos.at(ID_SERVO_4) = mCurrentPos.at(ID_SERVO_4) + 32;
            break;

        case 53: // kp 5
            mWantedPos.at(ID_SERVO_1) = 512;
            mWantedPos.at(ID_SERVO_2) = 512;
            mWantedPos.at(ID_SERVO_3) = 512;
            mWantedPos.at(ID_SERVO_4) = 512;
            break;

        case 0x1b: // Escape
        default:
            system("/bin/stty cooked");
            std::cout << "\033[10D"; // Get CLI cursor back to column 0
            std::cout << "> EXIT (key was: " << key << ")" << std::endl;
            running = false;
            break;
        }

        // Write goal position(s)
        for (std::vector <int>::iterator idsIterator = ids.begin(); idsIterator != ids.end(); idsIterator++)
        {
            // Moove needed?
            if (mCurrentPos.at(*idsIterator) != mWantedPos.at(*idsIterator))
            {
                dxl.setGoalPosition(*idsIterator, mWantedPos.at(*idsIterator));
            }
        }
    }

    std::cout << std::endl << "======== EXITING ========" << std::endl;

    // Close serial device(s)
    dxl.serialTerminate();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
