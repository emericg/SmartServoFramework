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
 * \file SerialPortLinux.cpp
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#if defined(__linux__) || defined(__gnu_linux)

#include "SerialPortLinux.h"

// Linux specifics
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

// "open" and "close" calls
#include <unistd.h>

// C++ standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <thread>

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    std::string basePort = "/dev/tty";

    // Used to validate existing serial port availability (currently disabled)
    //struct serial_struct serinfo;

    std::cout << "serialPortsScanner() [Linux variant]" << std::endl;

    // Serial ports from USB adapters (/dev/ttyUSB*) (ftdi or other chips)
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

            close(fd);
        }
    }

    // Serial ports from USB adapters (/dev/ttyACM*) ("abstract control model")
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

            close(fd);
        }
    }
/*
    // Regular Serial ports from motherboards (/dev/ttyS*)
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

            close(fd);
        }
    }

    // Regular Serial ports from motherboards (/dev/tty*)
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

            close(fd);
        }
    }
*/
    return retcode;
}

SerialPortLinux::SerialPortLinux(std::string &devicePath, const int baud, const int serialDevice, const int servoDevices):
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

        std::cout << "- Device name has been set to: '" << ttyDeviceName << "'" << std::endl;
        std::cout << "- Device node has been set to: '" << ttyDevicePath << "'" << std::endl;
        std::cout << "- Device baud rate has been set to: '" << ttyDeviceBaudRate << "'" << std::endl;
    }
/*
    // Autodetect latency time value for FTDI based devices
    if (serialDevice == SERIAL_USB2DYNAMIXEL || serialDevice == SERIAL_OTHER_FTDI)
    {
        setLatency(0);
        std::cout << "- Device latency time has been set to: '" << ttyDeviceLatencyTime << "'" << std::endl;
    }
*/
}

SerialPortLinux::~SerialPortLinux()
{
    closeLink();
}

void SerialPortLinux::setBaudRate(const int baud)
{
    // Get valid baud rate
    ttyDeviceBaudRate = checkBaudRate(baud);

    // Get <termios.h> baudrate flag
    ttyDeviceBaudRateFlag = convertBaudRateFlag(ttyDeviceBaudRate);

    // Compute the time needed to transfert one byte through the serial interface
    // (1000 / baudrate(= bit per msec)) * 10(= start bit + 8 data bit + stop bit)
    byteTransfertTime = (1000.0 / static_cast<double>(ttyDeviceBaudRate)) * 10.0;
}

static int rate_to_constant(int baudrate)
{
#define B(x) case x: return B##x

    switch (baudrate)
    {
        B(2400);    B(4800);    B(9600);    B(19200);   B(38400);
        B(57600);   B(115200);  B(230400);  B(460800);  B(500000);
        B(576000);  B(921600);  B(1000000); B(1152000); B(1500000);
        B(2000000); B(2500000); B(3000000); B(3500000); B(4000000);
        default:
            #undef B
            return 0;
    }
}

int SerialPortLinux::convertBaudRateFlag(int baudrate)
{
    int baudRateFlag = 0;

    // Set termios baudrate flag
    if (baudrate > 0)
    {
        // Try an exact match
        baudRateFlag = rate_to_constant(baudrate);

        if (baudRateFlag != 0)
        {
            //std::cout << "convertBaudRateFlag(" << baudrate << ") has been set to " << baudrate << " (exact match)" << std::endl;
        }
        else
        {
            int speeds[20] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
                              460800, 500000, 576000, 921600, 1000000, 1152000,
                              1500000, 2000000, 2500000, 3000000, 3500000, 4000000};

            // Try a "close enough" match (we allow ±1.5% mismatch)
            for (int i = 0; i < 20; i++)
            {
                if ((baudrate > (static_cast<double>(speeds[i]) * 98.5 / 100.0)) && (baudrate < (static_cast<double>(speeds[i]) * 101.5 / 100.0)))
                {
                    baudRateFlag = rate_to_constant(speeds[i]);
                    std::cout << "convertBaudRateFlag(" << baudrate << ") has been set to B" << speeds[i] << " (close enough match, ±1.5%)" << std::endl;

                    break;
                }
            }

            // Try a custom speed
            if (baudRateFlag == 0)
            {
                ttyCustomSpeed = true;
                baudRateFlag = B38400;
                std::cout << "convertBaudRateFlag(" << baudrate << ") has been set to B38400 (custom speed will be used)" << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Invalid baudrate, using default value of: B1000000" << std::endl;
    }

    // Fallback
    if (baudRateFlag == 0)
    {
        baudRateFlag = B1000000;
        std::cerr << "Unable to set baud speed at " << baudrate << ": too slow!" << std::endl;
        std::cerr << "Invalid baudrate, using default value of: B1000000" << std::endl;
    }

    return baudRateFlag;
}

bool SerialPortLinux::setLock()
{
    bool status = false;

    if (ttyDeviceLockPath != "null")
    {
        FILE *lock = std::fopen(ttyDeviceLockPath.c_str(), "w");
        if (lock)
        {
            std::stringstream ss;
            ss << std::this_thread::get_id();
            fputs(ss.str().c_str(), lock);
            ttyDeviceLocked = true;
            status = true;

            std::cout << "Lock set at: '" << ttyDeviceLockPath << "' for tid: '" << std::this_thread::get_id() << "'" << std::endl;
            std::fclose(lock);
        }
    }

    return status;
}

bool SerialPortLinux::isLocked()
{
    bool status = false;

    if (ttyDeviceLockPath != "null")
    {
        FILE *lock = std::fopen(ttyDeviceLockPath.c_str(), "r");
        if (lock)
        {
            char buf[16] = {0};
            if (std::fgets(buf, sizeof buf, lock) != NULL)
            {
                std::stringstream ss;
                ss << std::this_thread::get_id();

                if (strcmp(buf, ss.str().c_str()) != 0)
                {
                    std::cout << "Lock from another instance found at: '" << ttyDeviceLockPath << "'" << std::endl;
                    status = true;
                }
            }
            else
            {
                std::cout << "Lock found at: '" << ttyDeviceLockPath << "'" << std::endl;
                status = true;
            }

            std::fclose(lock);
        }
    }

    return status;
}

int SerialPortLinux::openLink()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Make sure no tty connection is already running (in that case, openLink() will do a reconnection)
    closeLink();

    // Check if another instance is using this port
    if (isLocked() == true)
    {
        std::cerr << "Cannot connect to serial port: '" << ttyDevicePath << "': interface is locked!" << std::endl;
        goto OPEN_LINK_ERROR;
    }

    // O_RDWR: ?
    // O_NOCTTY: if the named file is a terminal device, don't make it the controlling terminal for the process
    // O_NONBLOCK: non-blocking reads

    // Open tty device
    ttyDeviceFileDescriptor = open(ttyDevicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ttyDeviceFileDescriptor < 0)
    {
        std::cerr << "Unable to open device on serial port '" << ttyDevicePath << "'" << std::endl;
        goto OPEN_LINK_ERROR;
    }

    // ttyDeviceBaudRateFlag: flag from termios.h
    // CS8: setting the character size
    // CLOCAL: ?
    // CREAD: input can be read from the terminal
    // IGNPAR: ignore bit parity

    // Set newtio attributes
    tty.c_cflag     = ttyDeviceBaudRateFlag | CS8 | CLOCAL | CREAD;
    tty.c_iflag     = IGNPAR;
    tty.c_oflag     = 0;
    tty.c_lflag     = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;

    tcflush(ttyDeviceFileDescriptor, TCIFLUSH);
    tcsetattr(ttyDeviceFileDescriptor, TCSANOW, &tty);

    if (ttyDeviceBaudRate < 1)
    {
        std::cerr << "Unable to set baud rate to '" << ttyDeviceBaudRate << "': invalid!" << std::endl;
        goto OPEN_LINK_ERROR;
    }

    // Set custom serial infos?
    if (ttyCustomSpeed == true || ttyLowLatency == true)
    {
        struct serial_struct serinfo;
        memset(&serinfo, 0, sizeof(serinfo));

        // Get current serial_struct values
        if (ioctl(ttyDeviceFileDescriptor, TIOCGSERIAL, &serinfo) < 0)
        {
            std::cerr << "Cannot get serial infos structure from serial port: '" << ttyDevicePath << "'!" << std::endl;
            goto OPEN_LINK_ERROR;
        }

        if (ttyCustomSpeed == true)
        {
            serinfo.flags &= ~ASYNC_SPD_MASK;
            serinfo.flags |= ASYNC_SPD_CUST;
            serinfo.custom_divisor = serinfo.baud_base / ttyDeviceBaudRate;
            if (serinfo.custom_divisor < 1)
            {
                serinfo.custom_divisor = 1;
            }
        }

        if (ttyLowLatency == true)
        {
            serinfo.flags |= ASYNC_LOW_LATENCY;
        }

        // Set serial_struct
        if (ioctl(ttyDeviceFileDescriptor, TIOCSSERIAL, &serinfo) < 0)
        {
            std::cerr << "Cannot set serial infos structure with custom baud divisor (" << ttyDeviceBaudRate << ") to serial port: '" << ttyDevicePath << "'!" << std::endl;
            goto OPEN_LINK_ERROR;
        }
    }

    return 1;

OPEN_LINK_ERROR:
    closeLink();
    return 0;
}

bool SerialPortLinux::isOpen()
{
    bool status = false;

    if (ttyDeviceFileDescriptor > 0) // device has been openend?
    {
        if (fcntl(ttyDeviceFileDescriptor, F_GETFD) != -1 || errno != EBADF) // device is still connected?
        {
            status = true;
        }
    }

    return status;
}

void SerialPortLinux::closeLink()
{
    if (isOpen() == true)
    {
        this->flush();
        close(ttyDeviceFileDescriptor);
        ttyDeviceFileDescriptor = -1;

        if (ttyDeviceLocked == true && ttyDeviceLockPath != "null")
        {
            std::remove(ttyDeviceLockPath.c_str());
        }
    }
}

int SerialPortLinux::tx(unsigned char *packet, int packetLength)
{
    int writeStatus = -1;

    if (isOpen() == true)
    {
        if (packet != NULL && packetLength > 0)
        {
            writeStatus = write(ttyDeviceFileDescriptor, packet, packetLength);

            if (writeStatus < 0)
            {
                std::cerr << "Cannot write to serial port '" << ttyDevicePath << "': write() failed with error code '" << errno << "'" << std::endl;
            }
        }
        else
        {
            std::cerr << "Cannot write to serial port '" << ttyDevicePath << "': invalid packet buffer or size!" << std::endl;
        }
    }
    else
    {
        std::cerr << "Cannot write to serial port '" << ttyDevicePath << "': invalid device!" << std::endl;
    }

    return writeStatus;
}

int SerialPortLinux::rx(unsigned char *packet, int packetLength)
{
    int readStatus = -1;

    if (isOpen() == true)
    {
        if (packet != NULL && packetLength > 0)
        {
            memset(packet, 0, packetLength);
            readStatus = read(ttyDeviceFileDescriptor, packet, packetLength);

            if (readStatus < 0)
            {
                std::cerr << "Cannot read from serial port '" << ttyDevicePath << "': read() failed with error code '" << errno << "'" << std::endl;
            }
        }
        else
        {
            std::cerr << "Cannot read from serial port '" << ttyDevicePath << "': invalid packet buffer or size!" << std::endl;
        }
    }
    else
    {
        std::cerr << "Cannot read from serial port '" << ttyDevicePath << "': invalid device!" << std::endl;
    }

    return readStatus;
}

void SerialPortLinux::flush()
{
    if (isOpen() == true)
    {
        //TCIFLUSH: Flushes data received but not read.
        //TCOFLUSH: Flushes data written but not transmitted.
        //TCIOFLUSH: Flushes both data received but not read and data written but not transmitted.

        tcflush(ttyDeviceFileDescriptor, TCIFLUSH);
    }
}

double SerialPortLinux::getTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return (static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0);
}

bool SerialPortLinux::switchHighSpeed()
{
    bool status = false;

    // TODO // enables "ASYNC_LOW_LATENCY" flag
    // reduce the "/sys/bus/usb-serial/devices/ttyXXX/latency_timer" value

    return status;
}

void SerialPortLinux::setLatency(int latency)
{
    // "auto-detect" latency value if possible
    // note: this doesn't really produce expected results yet..
    if (latency == 0)
    {
        size_t lastbackslash = ttyDevicePath.find_last_of('/');

        if (lastbackslash != std::string::npos)
        {
            std::string latency_path = "/sys/bus/usb-serial/devices/" + ttyDevicePath.substr(lastbackslash+1) + "/latency_timer";
            std::ifstream latency_file(latency_path);

            if (latency_file.good())
            {
                char latency_value[8];
                latency_file.getline(latency_value, 8);
                latency = std::stoi(latency_value);
            }
            latency_file.close();
        }
        else
        {
            std::cerr << "Unable to find latency info for the current device: '" << ttyDevicePath.substr(lastbackslash+1) << "'" << std::endl;
        }
    }

    if (latency > 0 && latency < 128)
    {
        ttyDeviceLatencyTime = latency;
    }
    else
    {
        std::cerr << "Invalid latency value: '" << latency << "'', not in ]0;128[ range." << std::endl;
    }
}

void SerialPortLinux::setTimeOut(int packetLength)
{
    packetStartTime = getTime();
    packetWaitTime  = (byteTransfertTime * static_cast<double>(packetLength) + 2.0 * static_cast<double>(ttyDeviceLatencyTime));
}

void SerialPortLinux::setTimeOut(double msec)
{
    packetStartTime = getTime();
    packetWaitTime  = msec;
}

int SerialPortLinux::checkTimeOut()
{
    int status = 0;
    double time_elapsed = getTime() - packetStartTime;

    if (time_elapsed > packetWaitTime)
    {
        status = 1;
    }
    else if (time_elapsed < 0)
    {
        packetStartTime = getTime();
    }

    return status;
}

#endif /* __linux__ || __gnu_linux */
