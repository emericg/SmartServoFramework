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
 * \file SerialPortMacOS.cpp
 * \date 25/09/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#if defined(__APPLE__) || defined(__MACH__)

#include "SerialPortMacOS.h"

// Linux specifics
#include <fcntl.h>
#include <termios.h>
//#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <unistd.h>

// C++ standard libraries
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    std::string basePort = "/dev/tty";

    // Used to validate existing serial port availability (currently disabled)
    //struct serial_struct serinfo;

    std::cout << "serialPortsScanner() [MacOS variant]" << std::endl;

    // Serial ports from USB adapters (ftdi or other chips)
    for (int i = 0; i < 8; i++)
    {
        std::string port = basePort + "USB" + std::to_string(i);

        int fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
        if (fd > 0)
        {
            // Try to get serial infos
            //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
            {
                std::cout << "- Scanning for serial port on '" << port << "' > FOUND" << std::endl;
                availableSerialPorts.push_back(port);
                retcode++;
            }
        }
    }

    // Serial ports from USB adapters ("abstract control model")
    for (int i = 0; i < 8; i++)
    {
        std::string port = basePort + "ACM" + std::to_string(i);

        int fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
        if (fd > 0)
        {
            // Try to get serial infos
            //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
            {
                std::cout << "- Scanning for serial port on '" << port << "' > FOUND" << std::endl;
                availableSerialPorts.push_back(port);
                retcode++;
            }
        }
    }
/*
    // Regular Serial ports from motherboards (with "S" prefix)
    for (int i = 0; i < 8; i++)
    {
        std::string port = basePort + "S" + std::to_string(i);

        int fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
        if (fd > 0)
        {
            // Try to get serial infos
            //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
            {
                std::cout << "- Scanning for serial port on '" << port << "' > FOUND" << std::endl;
                availableSerialPorts.push_back(port);
                retcode++;
            }
        }
    }

    // Regular Serial ports from motherboards
    for (int i = 0; i < 8; i++)
    {
        std::string port = basePort + std::to_string(i);

        int fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
        if (fd > 0)
        {
            // Try to get serial infos
            //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
            {
                std::cout << "- Scanning for serial port on '" << port << "' > FOUND" << std::endl;
                availableSerialPorts.push_back(port);
                retcode++;
            }
        }
    }
*/
    return retcode;
}

SerialPortMacOS::SerialPortMacOS(std::string &deviceName, const int baud, const int serialDevice, const int servoDevices):
    SerialPort(serialDevice, servoDevices),
    ttyDeviceFileDescriptor(-1),
    ttyDeviceBaudRateFlag(B1000000),
    ttyCustomSpeed(false),
    ttyLowLatency(false)
{
    if (deviceName.empty() == 1 || deviceName == "auto")
    {
        ttyDeviceName = autoselectSerialPort();
    }
    else
    {
        ttyDeviceName = deviceName;
    }

    if (ttyDeviceName != "null")
    {
        std::cout << "- Device node has been set to: '" << ttyDeviceName << "'" << std::endl;

        setBaudRate(baud);
        std::cout << "- Device baud rate has been set to: '" << ttyDeviceBaudRate << "'" << std::endl;
    }
}

SerialPortMacOS::~SerialPortMacOS()
{
    closeLink();
}

void SerialPortMacOS::setLatency(int latency)
{
    // void
}

#endif /* defined(__APPLE__) || defined(__MACH__) */
