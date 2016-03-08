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
 * \file ex_basic_test.cpp
 * \date 29/04/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 *
 * Very simple test program: initialize the "Simple API" with serial port
 * auto-detection, scan this serial port for servo(s), then try to move the
 * servo at ID 1:
 *
 * - Use keys "up" and "down" to go back and forth from steps 256 to 768 (should
 * work on all servo series).
 * - Use keys "left" or "-" and "right" or "+" to substract or add 64 steps to
 * the current servo position.
 * - Use any other key to exit the program.
 */

// SmartServoFramework
#include "../src/DynamixelSimpleAPI.h"
#include "../src/HerkuleXSimpleAPI.h"

// C++ standard library
#include <iostream>

/* ************************************************************************** */

// ID of the servo you want to use in this test program:
#define ID_TEST     1

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

    std::cout << std::endl << "======== DEVICE SCANNING ========" << std::endl;

    // Scan serial port for servos, store results (device IDs found) inside a vector
    std::vector <int> ids = dxl.servoScan();

    std::cout << std::endl << "======== MAIN LOOP ========" << std::endl;

    std::cout << std::endl << "> Use the keypad (+/-) to move (press ESC or any other key to exit)" << std::endl;
    system("/bin/stty raw");

    // Init servo position
    int pos = dxl.readCurrentPosition(ID_TEST);

    bool running = true;
    while (running)
    {
        // Always get CLI cursor back to column 0
        std::cout << "\033[10D";

        // Move order?
        int key = getchar();
        switch (key)
        {
        case 43: // +
        case 55: // kp 7
            pos = dxl.readCurrentPosition(ID_TEST);
            dxl.setGoalPosition(ID_TEST, pos + 64);
            std::cout << "Current position is: " << pos << ". Moving to " << pos + 64 << std::endl;
            break;

        case 45: // -
        case 57: // kp 9
            pos = dxl.readCurrentPosition(ID_TEST);
            dxl.setGoalPosition(ID_TEST, pos - 64);
            std::cout << "Current position is: " << pos << ". Moving to " << pos - 64 << std::endl;
            break;

        case 52: // kp 4
            dxl.setGoalPosition(ID_TEST, 256);
            break;
        case 54: // kp 6
            dxl.setGoalPosition(ID_TEST, 768);
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

    // Close device(s)
    dxl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
