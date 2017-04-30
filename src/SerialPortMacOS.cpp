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
 * \date 16/08/2015
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#if defined(__APPLE__) || defined(__MACH__)

#include "SerialPortMacOS.h"
#include "minitraces.h"

// Unix
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>

// MacOSX
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#include <IOKit/serial/ioss.h>

// C++ standard libraries
#include <fstream>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// Device lock support
#define LOCK_TIOCEXCL
//#define LOCK_LOCKFILE

////////////////////////////////////////////////////////////////////////////////

// Custom speeds
#define B460800   460800
#define B500000   500000
#define B576000   576000
#define B921600   921600
#define B115200   115200
#define B1000000 1000000
#define B1152000 1152000
#define B1500000 1500000
#define B2000000 2000000
#define B2500000 2500000
#define B3000000 3000000
#define B3500000 3500000
#define B4000000 4000000

/*!
 * Find the first device that matches the callout device path MATCH_PATH.
 * If this is undefined, return the first device found.
 */
#define MATCH_PATH "/dev/cu.usbmodem"

/*!
 * Given an iterator across a set of modems, return the BSD path to the first one
 * with the callout device path matching MATCH_PATH if defined.
 * If MATCH_PATH is not defined, return the first device found.
 * If no modems are found the path name is set to an empty string.
 */
static bool getModemPath(io_iterator_t serialPortIterator, char *modemPath, CFIndex maxPathSize)
{
    bool status = false;
    io_object_t modemService;

    // Iterate across all modems found.
    while ((modemService = IOIteratorNext(serialPortIterator)))
    {
        // Get the callout device's path (/dev/cu.xxxxx). The callout device should almost always be
        // used: the dialin device (/dev/tty.xxxxx) would be used when monitoring a serial port for
        // incoming calls, e.g. a fax listener.

        CFTypeRef bsdPathAsCFString = IORegistryEntryCreateCFProperty(modemService,
                                                                      CFSTR(kIOCalloutDeviceKey),
                                                                      kCFAllocatorDefault, 0);
        if (bsdPathAsCFString)
        {
            // Convert the path from a CFString to a C string for
            // later use with the POSIX open() call.

            status = CFStringGetCString((CFStringRef)bsdPathAsCFString,
                                         modemPath, maxPathSize,
                                         kCFStringEncodingUTF8);
            CFRelease(bsdPathAsCFString);

#ifdef MATCH_PATH
            if (status)
            {
                if (strncmp(modemPath, MATCH_PATH, strlen(MATCH_PATH)) != 0)
                {
                    status = false;
                }
            }
#endif

            if (status)
            {
                status = true;
            }
        }

        IOObjectRelease(modemService);
    }

    return status;
}

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    kern_return_t kernResult = KERN_FAILURE;
    io_iterator_t serialPortIterator = 0;

    TRACE_INFO(SERIAL, "serialPortsScanner() [macOS variant]");

    // Serial devices are instances of class IOSerialBSDClient. Create a matching dictionary to find those.
    CFMutableDictionaryRef classesToMatch = nullptr;
    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);

    if (classesToMatch != nullptr)
    {
        // Look for devices that claim to be modems (will pick up usb adapters, but not regular RS232 ports)
        CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDModemType));

        // Get an iterator across all matching devices.
        kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, &serialPortIterator);

        if (kernResult != KERN_SUCCESS)
        {
            TRACE_ERROR(SERIAL, "IOServiceGetMatchingServices returned %d", kernResult);
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "IOServiceMatching returned a NULL dictionary.");
    }

    if (kernResult == KERN_SUCCESS)
    {
        char modemPath[MAXPATHLEN] = "\0";

        if (getModemPath(serialPortIterator, modemPath, sizeof(modemPath)) == true)
        {
            // Test the port?
            //int fileDescriptor = openSerialPort(bsdPath);
            //if (-1 == fileDescriptor)
            //{
            //    return EX_IOERR;
            //}

            TRACE_INFO(SERIAL, "- Scanning for serial port on '%s' > FOUND", modemPath);
            retcode++;

            // Add it to the list
            std::string portPath = modemPath;
            availableSerialPorts.push_back(portPath);
        }
        else
        {
            TRACE_ERROR(SERIAL, "Could not get path for modem.");
        }
    }
    else
    {
        TRACE_WARNING(SERIAL, "No modem port were found.");
    }

    IOObjectRelease(serialPortIterator);

    return retcode;
}

SerialPortMacOS::SerialPortMacOS(std::string &devicePath, const int baud, const int serialDevice, const int servoDevices):
    SerialPort(serialDevice, servoDevices),
    ttyDeviceFileDescriptor(-1),
    ttyDeviceBaudRateFlag(B1000000),
    ttyCustomSpeed(false)
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

        TRACE_INFO(SERIAL, "- Device name has been set to: '%s'", ttyDeviceName.c_str());
        TRACE_INFO(SERIAL, "- Device node has been set to: '%s'", ttyDevicePath.c_str());
        TRACE_INFO(SERIAL, "- Device baud rate has been set to: '%i'", ttyDeviceBaudRate);
    }
}

SerialPortMacOS::~SerialPortMacOS()
{
    closeLink();
}

void SerialPortMacOS::setBaudRate(const int baud)
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

int SerialPortMacOS::convertBaudRateFlag(int baudrate)
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

bool SerialPortMacOS::isLocked()
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

#ifdef LOCK_TIOCEXCL
    //
#endif

    return status;
}

bool SerialPortMacOS::setLock()
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

#ifdef LOCK_TIOCEXCL
        // Note that open() follows POSIX semantics: multiple open() calls to the same file will succeed
        // unless the TIOCEXCL ioctl is issued. This will prevent additional opens except by root-owned processes.
        if (ioctl(ttyDeviceFileDescriptor, TIOCEXCL) == -1)
        {
            TRACE_ERROR(SERIAL, "- Unable to set TIOCEXCL lock for '%s': %s(%d).",
                        ttyDevicePath.c_str(), strerror(errno), errno);
        }
        else
        {
            status = true;
            TRACE_INFO(SERIAL, "- TIOCEXCL lock set for: '%s'", ttyDeviceLockPath.c_str());
        }
#endif
    }

    return status;
}

bool SerialPortMacOS::removeLock()
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

    return status;
}

int SerialPortMacOS::openLink()
{
    struct termios tty;
    //memset(&tty, 0, sizeof(tty));

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
    // O_EXLOCK: Acquire an exclusive lock on the file.

    ttyDeviceFileDescriptor = open(ttyDevicePath.c_str(), O_RDWR | O_NOCTTY);
    if (ttyDeviceFileDescriptor < 0)
    {
        TRACE_ERROR(SERIAL, "Unable to open device on serial port: '%s', %s(%d)",
                    ttyDevicePath.c_str(), strerror(errno), errno);
        goto OPEN_LINK_ERROR;
    }

    // Lock device
    setLock();

    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(ttyDeviceFileDescriptor, &tty) == -1)
    {
        TRACE_ERROR(SERIAL, "Error getting tty attributes %s - %s(%d).",
                    ttyDevicePath.c_str(), strerror(errno), errno);
        goto OPEN_LINK_ERROR;
    }

    // ttyDeviceBaudRateFlag: flag from termios.h
    // CS8: setting the character size
    // CLOCAL: ?
    // CREAD: input can be read from the terminal
    // IGNPAR: ignore bit parity

    // Set newtio attributes
    tty.c_cflag     = CS8 | CLOCAL | CREAD;
    tty.c_iflag     = IGNPAR;
    tty.c_oflag     = 0;
    tty.c_lflag     = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;

    // This used to be necessary for the serial port to connect with older versions
    // of macOS (< 10.12). Try uncommenting this if you experience problems...
    //sleep(1);

    tcflush(ttyDeviceFileDescriptor, TCIFLUSH);
    // Cause the new options to take effect immediately.
    if (tcsetattr(ttyDeviceFileDescriptor, TCSANOW, &tty) == -1)
    {
       TRACE_ERROR(SERIAL, "Error setting tty attributes %s - %s(%d).",
                   ttyDevicePath.c_str(), strerror(errno), errno);
       goto OPEN_LINK_ERROR;
    }

    TRACE_1(SERIAL, "Current input baud rate is %d", static_cast<int>(cfgetispeed(&tty)));
    TRACE_1(SERIAL, "Current output baud rate is %d", static_cast<int>(cfgetospeed(&tty)));

    // This used to be necessary for the serial port to connect with older versions
    // of macOS (< 10.12). Try uncommenting this if you experience problems...
    //sleep(1);

    if (ttyDeviceBaudRate < 1)
    {
        TRACE_ERROR(SERIAL, "Unable to set baud rate to '%i'bps: invalid value", ttyDeviceBaudRate);
        goto OPEN_LINK_ERROR;
    }

    // Set custom serial infos?
    {
        cfsetspeed(&tty, ttyDeviceBaudRateFlag);

        // The IOSSIOSPEED ioctl can be used to set arbitrary baud rates
        // other than those specified by POSIX. The driver for the underlying serial hardware
        // ultimately determines which baud rates can be used. This ioctl sets both the input
        // and output speed.

        if (ioctl(ttyDeviceFileDescriptor, IOSSIOSPEED, &(ttyDeviceBaudRate)) == -1)
        {
            TRACE_ERROR(SERIAL, "Error calling ioctl(..., IOSSIOSPEED, ...) %s - %s(%d).",
                        ttyDevicePath.c_str(), strerror(errno), errno);
        }

        // Print the new input and output baud rates. Note that the IOSSIOSPEED ioctl interacts with the serial driver
        // directly bypassing the termios struct. This means that the following two calls will not be able to read
        // the current baud rate if the IOSSIOSPEED ioctl was used but will instead return the speed set by the last call
        // to cfsetspeed.

        TRACE_1(SERIAL, "Input baud rate changed to %d", static_cast<int>(cfgetispeed(&tty)));
        TRACE_1(SERIAL, "Output baud rate changed to %d", static_cast<int>(cfgetospeed(&tty)));

        unsigned long mics = 1UL;
        if (ioctl(ttyDeviceFileDescriptor, IOSSDATALAT, &mics) == -1)
        {
            // set latency to 1 microsecond
            TRACE_ERROR(SERIAL, "Error setting read latency %s - %s(%d).",
                        ttyDevicePath.c_str(), strerror(errno), errno);
            goto OPEN_LINK_ERROR;
        }
    }
/*
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
*/
    return 1;

OPEN_LINK_ERROR:
    closeLink();
    return 0;

OPEN_LINK_LOCKED:
    closeLink();
    return -1;
}

bool SerialPortMacOS::isOpen()
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

void SerialPortMacOS::closeLink()
{
    if (isOpen() == true)
    {
        this->flush();
        close(ttyDeviceFileDescriptor);
        ttyDeviceFileDescriptor = -1;

        removeLock();
    }
}

int SerialPortMacOS::tx(unsigned char *packet, int packetLength)
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

int SerialPortMacOS::rx(unsigned char *packet, int packetLength)
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

void SerialPortMacOS::flush()
{
    if (isOpen() == true)
    {
/*
        // Block until all written output has been sent from the device.
        // Note that this call is simply passed on to the serial device driver.
        if (tcdrain(ttyDeviceFileDescriptor) == -1)
        {
            TRACE_ERROR(SERIAL, "Error waiting for drain - %s(%d).", strerror(errno), errno);
        }
*/
        //TCIFLUSH: Flushes data received but not read.
        //TCOFLUSH: Flushes data written but not transmitted.
        //TCIOFLUSH: Flushes both data received but not read and data written but not transmitted.

        tcflush(ttyDeviceFileDescriptor, TCIFLUSH);
    }
}

double SerialPortMacOS::getTime()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);

    return (static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0);
}

void SerialPortMacOS::setTimeOut(int packetLength)
{
    packetStartTime = getTime();
    packetWaitTime  = (byteTransfertTime * static_cast<double>(packetLength) + 2.0 * static_cast<double>(ttyDeviceLatencyTime));
}

void SerialPortMacOS::setTimeOut(double msec)
{
    packetStartTime = getTime();
    packetWaitTime  = msec;
}

int SerialPortMacOS::checkTimeOut()
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

#endif /* defined(__APPLE__) || defined(__MACH__) */
