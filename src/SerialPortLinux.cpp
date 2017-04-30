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
#include "minitraces.h"

// Linux specifics
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

// Device lock support
//#define LOCK_FLOCK
//#define LOCK_LOCKFILE
//#define LOCK_LOCKDEV

#ifdef LOCK_FLOCK
#include <sys/file.h>
#endif
#ifdef LOCK_LOCKDEV
#include <lockdev.h>
#endif

// "open" and "close" calls
#include <unistd.h>

// C++ standard libraries
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <thread>

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    std::string portBase = "/dev/tty";
    std::string portVariations[4] = {"USB", "ACM", "S", ""};

    TRACE_INFO(SERIAL, "serialPortsScanner() [Linux variant]");

    // Serial ports from USB adapters (/dev/ttyUSB*) (ftdi or other chips)
    // Serial ports from USB adapters (/dev/ttyACM*) ("abstract control model")
    // Regular Serial ports from motherboards (/dev/ttyS*)
    // Regular Serial ports from motherboards (/dev/tty*)

    for (int i = 0; i < 2; i++)
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
//#ifdef LOCK_LOCKDEV
//                if (dev_testlock(portName.c_str()) == 0)
//#endif
                {
                    // Used to validate existing serial port availability (currently disabled)
                    //struct serial_struct serinfo;
                    //if (ioctl(fd, TIOCGSERIAL, &serinfo) > -1)
                    {
                        TRACE_INFO(SERIAL, "- Scanning for serial port on '%s' > FOUND", portPath.c_str());
                        availableSerialPorts.push_back(portPath);
                        retcode++;
                    }
                }
//#ifdef LOCK_LOCKDEV
//                else
//                {
//                    TRACE_WARNING(SERIAL, "- Scanning for serial port on '%s' > LOCKED", portPath.c_str());
//                }
//#endif
                close(fd);
            }
        }
    }

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

            //ttyDeviceLockPath = "/var/lock/lockdev/LCK..";
            ttyDeviceLockPath = "/tmp/LCK..";
            ttyDeviceLockPath += ttyDeviceName;
        }

        setBaudRate(baud);

        TRACE_INFO(SERIAL, "- Device name has been set to: '%s'", ttyDeviceName.c_str());
        TRACE_INFO(SERIAL, "- Device node has been set to: '%s'", ttyDevicePath.c_str());
        TRACE_INFO(SERIAL, "- Device baud rate has been set to: '%i'", ttyDeviceBaudRate);
    }
/*
    // Autodetect latency time value for FTDI based devices
    if (serialDevice == SERIAL_USB2DYNAMIXEL || serialDevice == SERIAL_OTHER_FTDI)
    {
        setLatency(0);
        TRACE_INFO(SERIAL, "- Device latency time has been set to: '%i'", ttyDeviceLatencyTime);
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
            TRACE_1(SERIAL, "convertBaudRateFlag(%i) has been set to %i (exact match)", baudrate, baudrate);
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
                    TRACE_WARNING(SERIAL, "convertBaudRateFlag(%i) has been set to B%i (close enough match, ±1.5%)", baudrate, speeds[i]);
                    break;
                }
            }

            // Try a custom speed
            if (baudRateFlag == 0)
            {
                ttyCustomSpeed = true;
                baudRateFlag = B38400;
                TRACE_WARNING(SERIAL, "convertBaudRateFlag(%i) has been set to B38400 (custom speed will be used)", baudrate);
            }
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Invalid baudrate, using default value of: B1000000");
    }

    // Fallback
    if (baudRateFlag == 0)
    {
        baudRateFlag = B1000000;
        TRACE_ERROR(SERIAL, "Unable to set baud speed at %i: too slow!", baudrate);
        TRACE_ERROR(SERIAL, "Invalid baudrate, using default value of: B1000000");
    }

    return baudRateFlag;
}

bool SerialPortLinux::isLocked()
{
    bool status = false;

#ifdef LOCK_LOCKFILE
    if (ttyDeviceLockPath != "null" && ttyDeviceLockPath.empty() == false)
    {
        FILE *lock = std::fopen(ttyDeviceLockPath.c_str(), "r");
        if (lock)
        {
            char buf[16] = {0};
            if (std::fgets(buf, sizeof buf, lock) != nullptr)
            {
                std::stringstream ss;
                ss << std::this_thread::get_id();

                if (strcmp(buf, ss.str().c_str()) != 0)
                {
                    TRACE_WARNING(SERIAL, "- Device '%s' is LOCKED", ttyDevicePath.c_str());
                    TRACE_1(SERIAL, "Lock from another instance or program found at: '%s'", ttyDeviceLockPath.c_str());
                    status = true;
                }
            }
            else
            {
                TRACE_WARNING(SERIAL, "- Device '%s' is LOCKED", ttyDevicePath.c_str());
                TRACE_1(SERIAL, "Lock found at: '%s'", ttyDeviceLockPath.c_str());
                status = true;
            }

            std::fclose(lock);
        }
    }
#endif

#ifdef LOCK_LOCKDEV
    if (dev_testlock(ttyDeviceName.c_str()) != 0)
    {
        status = true;
        TRACE_WARNING(SERIAL, "- Device '%s' is LOCKED", ttyDevicePath.c_str());
    }
#endif

#ifdef LOCK_FLOCK
    int fd = open(ttyDevicePath.c_str(), O_RDONLY);
    if (fd > 0)
    {
        close(fd);
    }
    else
    {
        status = true;
        TRACE_WARNING(SERIAL, "- Device '%s' is LOCKED", ttyDevicePath.c_str());
    }
#endif

    return status;
}

bool SerialPortLinux::setLock()
{
    bool status = false;

    // First, check if device locking has been enabled for this link
    if (ttyDeviceLockMode > 0)
    {
#ifdef LOCK_LOCKFILE
        if (ttyDeviceLockPath != "null")
        {
            // Write file
            FILE *lock = std::fopen(ttyDeviceLockPath.c_str(), "w");
            if (lock)
            {
                // Write PID inside
                std::stringstream ss;
                ss << std::this_thread::get_id(); // use pid_t/getpid() instead?
                std::fputs(ss.str().c_str(), lock);
                std::fclose(lock);

                status = true;
                TRACE_INFO(SERIAL, "- Device lock set at: '%s' for tid: '%i'", ttyDeviceLockPath.c_str(), std::this_thread::get_id());
            }
            else
            {
                TRACE_ERROR(SERIAL, "- Unable to set lockfile '%s':  do you have necessary permissions to write in this directory?", ttyDeviceLockPath.c_str());
            }
        }
#endif

#ifdef LOCK_LOCKDEV
        if (dev_lock(ttyDeviceName.c_str()) == 0)
        {
            status = true;
            TRACE_INFO(SERIAL, "- Device lock set at: '%s' for tid: '%i'", ttyDeviceLockPath.c_str(), std::this_thread::get_id());
        }
        else
        {
            TRACE_ERROR(SERIAL, "- Unable to use lockdev for '%s':  do you have necessary permissions?", ttyDevicePath.c_str());
        }
#endif

#ifdef LOCK_FLOCK
        //LOCK_EX: Place an exclusive lock. Only one process may hold an exclusive lock for a given file at a given time.
        //LOCK_NB: A call to flock() may block if an incompatible lock is held by another process. To make a nonblocking request, include LOCK_NB (by ORing) with any of the above operations.

        if (flock(ttyDeviceFileDescriptor, LOCK_EX | LOCK_NB) == 0)
        {
            status = true;
            TRACE_INFO(SERIAL, "- Device lock set for '%s'", ttyDevicePath.c_str());
        }
        else
        {
            TRACE_ERROR(SERIAL, "- Unable to use flock for '%s':  do you have necessary permissions?", ttyDevicePath.c_str());

            if (errno == EBADF)
            {
                TRACE_ERROR(SERIAL, "EBADF");
            }
            if (errno == EINTR)
            {
                TRACE_ERROR(SERIAL, "EINTR");
            }
            if (errno == EINVAL)
            {
                TRACE_ERROR(SERIAL, "EINVAL");
            }
            if (errno == ENOLCK)
            {
                TRACE_ERROR(SERIAL, "ENOLCK");
            }
            if (errno == EWOULDBLOCK)
            {
                TRACE_ERROR(SERIAL, "EWOULDBLOCK");
            }
        }
#endif
    }

    return status;
}

bool SerialPortLinux::removeLock()
{
    bool status = false;

#ifdef LOCK_LOCKFILE
    if (ttyDeviceLockPath != "null" && ttyDeviceLockPath.empty() == false)
    {
        if (std::remove(ttyDeviceLockPath.c_str()) == 0)
        {
            status = true;
            TRACE_INFO(SERIAL, "Lock removed for device '%s'", ttyDevicePath.c_str());
        }
        else
        {
            TRACE_ERROR(SERIAL, "Error when unlocking port '%s'!", ttyDevicePath.c_str());
        }
    }
#endif

#ifdef LOCK_LOCKDEV
    pid_t pid = getpid();
    if (dev_unlock(ttyDeviceName.c_str(), pid) == 0)
    {
        status = true;
        TRACE_INFO(SERIAL, "Lock removed for device '%s'", ttyDevicePath.c_str());
    }
    else
    {
        TRACE_ERROR(SERIAL, "Error when unlocking port '%s'! error: %i!", ttyDevicePath.c_str(), errno);
    }
#endif

#ifdef LOCK_FLOCK
    //LOCK_UN: Remove an existing lock held by this process.
    //LOCK_NB: A call to flock() may block if an incompatible lock is held by another process. To make a nonblocking request, include LOCK_NB (by ORing) with any of the above operations.

    if (flock(ttyDeviceFileDescriptor, LOCK_UN | LOCK_NB) == 0)
    {
        status = true;
        TRACE_INFO(SERIAL, "Lock removed for device '%s'", ttyDevicePath);
    }
    else
    {
        TRACE_ERROR(SERIAL, "Error when unlocking port '%s'! error: %i!", ttyDevicePath, errno);
    }
#endif

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
        TRACE_ERROR(SERIAL, "Cannot connect to serial port: '%s': interface is locked!", ttyDevicePath.c_str());
        goto OPEN_LINK_LOCKED;
    }

    // Open tty device
    // O_RDWR: Request opening the file read/write
    // O_NOCTTY: If the named file is a terminal device, don't make it the controlling terminal for the process
    // O_EXLOCK: Acquire an exclusive lock on the file. Note: not available on linux.

    ttyDeviceFileDescriptor = open(ttyDevicePath.c_str(), O_RDWR | O_NOCTTY);
    if (ttyDeviceFileDescriptor < 0)
    {
        TRACE_ERROR(SERIAL, "Unable to open device on serial port: '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }

    // Lock device
    setLock();

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
        TRACE_ERROR(SERIAL, "Unable to set baud rate to '%i'bps: invalid value", ttyDeviceBaudRate);
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
            TRACE_ERROR(SERIAL, "Cannot get serial infos structure from serial port: '%s'", ttyDevicePath.c_str());
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
            TRACE_ERROR(SERIAL, "Cannot set serial infos structure with custom baud divisor (%s) to serial port: '%s'", ttyDeviceBaudRate, ttyDevicePath.c_str());
            goto OPEN_LINK_ERROR;
        }
    }

    return 1;

OPEN_LINK_ERROR:
    closeLink();
    return 0;

OPEN_LINK_LOCKED:
    closeLink();
    return -1;
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

        removeLock();
    }
}

int SerialPortLinux::tx(unsigned char *packet, int packetLength)
{
    int writeStatus = -1;

    if (isOpen() == true)
    {
        if (packet != nullptr && packetLength > 0)
        {
            writeStatus = write(ttyDeviceFileDescriptor, packet, packetLength);

            if (writeStatus < 0)
            {
                TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': write() failed with error code '%i'!", ttyDevicePath.c_str(), errno);
            }
        }
        else
        {
            TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': invalid packet buffer or size!", ttyDevicePath.c_str());
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': invalid device!", ttyDevicePath.c_str());
    }

    return writeStatus;
}

int SerialPortLinux::rx(unsigned char *packet, int packetLength)
{
    int readStatus = -1;

    if (isOpen() == true)
    {
        if (packet != nullptr && packetLength > 0)
        {
            memset(packet, 0, packetLength);
            readStatus = read(ttyDeviceFileDescriptor, packet, packetLength);

            if (readStatus < 0)
            {
                TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': read() failed with error code '%i'!", ttyDevicePath.c_str(), errno);
            }
        }
        else
        {
            TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': invalid packet buffer or size!", ttyDevicePath.c_str());
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': invalid device!", ttyDevicePath.c_str());
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
    gettimeofday(&tv, nullptr);

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
            TRACE_ERROR(SERIAL, "Unable to find latency info for the current device: '%s'", ttyDevicePath.substr(lastbackslash+1).c_str());
        }
    }

    if (latency > 0 && latency < 128)
    {
        ttyDeviceLatencyTime = latency;
    }
    else
    {
        TRACE_WARNING(SERIAL, "Invalid latency value: '%i', not in ]0;128[ range.", latency);
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

#endif // __linux__ || __gnu_linux
