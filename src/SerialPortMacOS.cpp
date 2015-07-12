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
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#if defined(__APPLE__) || defined(__MACH__)

#include "SerialPortMacOS.h"
#include "minitraces.h"

// Linux specifics
#include <fcntl.h>
#include <termios.h>
//#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

// "open" and "close" calls
#include <unistd.h>

// C++ standard libraries
#include <fstream>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    std::string portBase = "/dev/";
    std::string portVariations[5] = {"cu", "ttyUSB", "ttyACM", "ttyS", ""};

    TRACE_INFO(SERIAL, "serialPortsScanner() [MacOS variant]\n");

    // MacOSX variant? (/dev/cu*)
    // Serial ports from USB adapters (/dev/ttyUSB*) (ftdi or other chips)
    // Serial ports from USB adapters (/dev/ttyACM*) ("abstract control model")
    // Regular Serial ports from motherboards (/dev/ttyS*)
    // Regular Serial ports from motherboards (/dev/tty*)

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            std::string portPath = portBase + portVariations[i] + std::to_string(j);
            std::string portName = "tty" + portVariations[i] + std::to_string(j);

            // Open arguments:
            // O_RDONLY: Request opening the file read/write
            // O_NOCTTY: If the named file is a terminal device, don't make it the controlling terminal for the process

            int fd = open(portPath.c_str(), O_RDONLY | O_NOCTTY);
            if (fd > 0)
            {
#ifdef LOCK_LOCKDEV
                if (dev_testlock(portName.c_str()) == 0)
#endif
                {
                    // Used to validate existing serial port availability (currently disabled)
                    //struct serial_struct serinfo;
                    //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
                    {
                        TRACE_INFO(SERIAL, "- Scanning for serial port on '%s' > FOUND\n", portPath.c_str());
                        availableSerialPorts.push_back(portPath);
                        retcode++;
                    }
                }
#ifdef LOCK_LOCKDEV
                else
                {
                    TRACE_WARNING(SERIAL, "- Scanning for serial port on '%s' > LOCKED\n", portPath.c_str());
                }
#endif
                close(fd);
            }
        }
    }

    return retcode;
}

SerialPortMacOS::SerialPortMacOS(std::string &devicePath, const int baud, const int serialDevice, const int servoDevices):
    SerialPort(serialDevice, servoDevices),
    ttyDeviceFileDescriptor(-1),
    ttyDeviceBaudRateFlag(B1000000),
    ttyCustomSpeed(false),
    ttyLowLatency(false)
{
    if (devicePath.empty() == 1 || devicePath == "auto")
    {
        ttyDevicePath = autoselectSerialPort();
    }
    else
    {
        ttyDevicePath = devicePath;
    }

    if (ttyDevicePath != "null")
    {
        size_t found = ttyDevicePath.rfind("/");
        if (found != std::string::npos && found != ttyDevicePath.size())
        {
            ttyDeviceName = ttyDevicePath.substr(found + 1);

            ttyDeviceLockPath = "/tmp/";
            ttyDeviceLockPath += ttyDeviceName;
            ttyDeviceLockPath += ".lock";
        }

        setBaudRate(baud);

        TRACE_INFO(SERIAL, "- Device name has been set to: '%s'\n", ttyDeviceName.c_str());
        TRACE_INFO(SERIAL, "- Device node has been set to: '%s'\n", ttyDevicePath.c_str());
        TRACE_INFO(SERIAL, "- Device baud rate has been set to: '%i'\n", ttyDeviceBaudRate);
    }
}

SerialPortMacOS::~SerialPortMacOS()
{
    closeLink();
}

void SerialPortMacOS::setLatency(int latency)
{
    // TODO
}

#endif /* defined(__APPLE__) || defined(__MACH__) */
