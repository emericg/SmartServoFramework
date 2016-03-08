/*!
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 INRIA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * \file ex_simple_threaded.cpp
 * \date 17/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 *
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
 *
 * The only difference between this program and ex_simple.cpp is that the reads
 * and writes of servo positions is now done in a seperate threads than the control
 * keys handling.
 */

// SmartServoFramework
#include "../src/DynamixelSimpleAPI.h"
#include "../src/HerkuleXSimpleAPI.h"

// C++ standard libraries
#include <iostream>
#include <map>
#include <thread>

/* ************************************************************************** */

// ID of the servos you want to use in this test program:
#define ID_SERVO_1       1
#define ID_SERVO_2       2
#define ID_SERVO_3       3
#define ID_SERVO_4       4

/* ************************************************************************** */

void read_loop(DynamixelSimpleAPI &dxl,
               std::vector <int> &ids,
               std::map <int, int> &mCurrentPos,
               std::map <int, int> &mWantedPos)
{
    while (true)
    {
        // Read current position(s)
        for (std::vector <int>::iterator idsIterator = ids.begin(); idsIterator != ids.end(); idsIterator++)
        {
            mCurrentPos.at(*idsIterator) = dxl.readCurrentPosition(*idsIterator);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void write_loop(DynamixelSimpleAPI &dxl,
                std::vector <int> &ids,
                std::map <int, int> &mCurrentPos,
                std::map <int, int> &mWantedPos)
{
    while (true)
    {
        // Write goal position(s)
        for (std::vector <int>::iterator idsIterator = ids.begin(); idsIterator != ids.end(); idsIterator++)
        {
            dxl.setGoalPosition(*idsIterator, mWantedPos.at(*idsIterator));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

/* ************************************************************************** */

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======== Smart Servo Framework Tester ========" << std::endl;

    // Initialize a Dynamixel "Simple API" instance
    DynamixelSimpleAPI dxl;

    // Initialize a serial link for the SimpleAPI
    // You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
    // Note: serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.
    std::string deviceName = "auto";
    if (dxl.connect(deviceName, 1) == 0)
    {
        std::cerr << "> Failed to open a serial link for our SimpleAPI! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Scan serial port for servos, store results (device IDs found) inside a vector
    std::vector <int> ids = dxl.servoScan();
    std::map <int, int> mCurrentPos;
    std::map <int, int> mWantedPos;

    // Read initial position(s) for each servo found
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

    std::cout << std::endl << "======== BASIC TRHEAD TESTING ========" << std::endl;

    std::thread t2(read_loop, std::ref(dxl), std::ref(ids), std::ref(mCurrentPos), std::ref(mWantedPos));
    std::thread t3(write_loop, std::ref(dxl), std::ref(ids), std::ref(mCurrentPos), std::ref(mWantedPos));

    std::cout << std::endl << "======== MAIN LOOP ========" << std::endl;

    std::cout << std::endl << "> Use the keypad to move (press ESC or any other key to exit)" << std::endl;
    system("/bin/stty raw");

    bool running = true;
    while (running)
    {
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
    }

    std::cout << std::endl << "======== EXITING ========" << std::endl;

    // Join threads in order to stop them properly
    t2.join();
    t3.join();

    // Close serial device(s)
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
