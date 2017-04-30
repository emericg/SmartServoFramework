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
 * \file SerialPort.cpp
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "SerialPort.h"
#include "DynamixelTools.h"
#include "HerkuleXTools.h"
#include "minitraces.h"

// Include the OS specific serialPortsScanner()
#include "SerialPortQt.h"
#include "SerialPortLinux.h"
#include "SerialPortMacOS.h"
#include "SerialPortWindows.h"

SerialPort::SerialPort(const int serialDevice, const int servoDevices):
    ttyDeviceName("null"),
    ttyDevicePath("null"),
    ttyDeviceBaudRate(1000000),
    ttyDeviceLatencyTime(LATENCY_TIME_DEFAULT),
    ttyDeviceLockMode(1),
    ttyDeviceLockPath("null"),
    serialDevice(serialDevice),
    servoDevices(servoDevices),
    packetStartTime(0.0),
    packetWaitTime(0.0),
    byteTransfertTime(0.0)
{
    //
}

SerialPort::~SerialPort()
{
    //
}

int SerialPort::checkBaudRate(const int baud)
{
    int baudRate = 57600;

    // Set default baudrates
    if (servoDevices >= SERVO_HERKULEX)
    {
        // Default baudrate for HerkuleX devices
        baudRate = 115200;
    }
    else if (servoDevices >= SERVO_DYNAMIXEL)
    {
        if (servoDevices >= SERVO_PRO)
        {
            // Default baudrate for Dynamixel PRO devices
            baudRate = 57600;
        }
        else if (servoDevices >= SERVO_X)
        {
            // Default baudrate for Dynamixel X devices
            baudRate = 57600;
        }
        else if (servoDevices >= SERVO_XL)
        {
            // Default baudrate for Dynamixel XL-320 devices
            baudRate = 1000000;
        }
        else if (servoDevices >= SERVO_RX)
        {
            // Default baudrate for Dynamixel RX, EX and MX devices
            baudRate = 57600;
        }
        else if (servoDevices >= SERVO_AX)
        {
            // Default baudrate for Dynamixel AX devices
            baudRate = 1000000;
        }
        else if (servoDevices >= SERVO_DX)
        {
            // Default baudrate for Dynamixel DX devices
            baudRate = 57600;
        }
    }

    if (baud >= 0)
    {
        if (servoDevices >= SERVO_HERKULEX)
        {
            if (baud < 255)
            {
                // We have a 'baudnum' not a 'baudrate'
                baudRate = hkx_get_baudrate(baud);
            }
            else if (baud >= 57600 && baud <= 1000000)
            {
                // We already have a 'valid' baudrate
                baudRate = baud;
            }
            else
            {
                TRACE_ERROR(SERIAL, "Invalid baudrate '%i' bps for a HerkuleX device, using default baudrate value of: '%i' bps", baud, baudRate);
            }
        }
        else if (servoDevices >= SERVO_DYNAMIXEL)
        {
            if (baud < 255)
            {
                // We have a 'baudnum' not a 'baudrate'
                baudRate = dxl_get_baudrate(baud, servoDevices);
            }
            else if (baud >= 2400 && baud <= 10500000)
            {
                // We already have a 'valid' baudrate
                baudRate = baud;
            }
            else
            {
                TRACE_ERROR(SERIAL, "Invalid baudrate '%i' bps for a Dynamixel device, using default baudrate value of: '%i' bps", baud, baudRate);
            }
        }
        else
        {
            if (baud >= 2400 && baud <= 1000000)
            {
                // We already have a 'valid' baudrate
                baudRate = baud;
            }
            else
            {
                TRACE_ERROR(SERIAL, "Invalid device class '%i' with baudrate '%i' bps, using default baudrate value of: '%i'bps", servoDevices, baud, baudRate);
            }
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Invalid baudrate '%i' bps, using default baudrate value of: '%i' bps", baud, baudRate);
    }

    // Apply bandwith restriction depending on the adapter chip
    if (serialDevice == SERIAL_USB2DYNAMIXEL || serialDevice == SERIAL_OTHER_FTDI)
    {
        if (baudRate > 4500000)
        {
            baudRate = 4500000;
            TRACE_ERROR(SERIAL, "Invalid baudrate ('%i' > 4500000): too high for SERIAL_USB2DYNAMIXEL or or FTDI based device, using default baudrate value of: '1000000'", baud);
        }
    }
    else if (serialDevice == SERIAL_USB2AX)
    {
        if (baudRate > 1000000)
        {
            baudRate = 1000000;
            TRACE_ERROR(SERIAL, "Invalid baudrate ('%i' > 1000000): too high for USB2AX device, using default baudrate value of: '1000000'", baud);
        }
    }
    else if (serialDevice == SERIAL_ZIG100)
    {
        if (baudRate > 115200)
        {
            baudRate = 115200;
            TRACE_ERROR(SERIAL, "Invalid baudrate ('%i' > 115200): too high for ZIG-100/110A device, using fallback baudrate value of: '115200'", baud);
        }
    }

    return baudRate;
}

std::string SerialPort::autoselectSerialPort()
{
    std::vector <std::string> availableSerialPorts = scanSerialPorts();

    if (availableSerialPorts.size() > 0)
    {
        return availableSerialPorts.at(0);
    }
    else
    {
        return "null";
    }
}

std::vector <std::string> SerialPort::scanSerialPorts()
{
    std::vector <std::string> availableSerialPorts;

#if defined(FEATURE_QTSERIAL)
    if (serialPortsScannerQt(availableSerialPorts) == 0)
#else
    // This function is OS specific
    if (serialPortsScanner(availableSerialPorts) == 0)
#endif
    {
        TRACE_WARNING(SERIAL, "No serial ports found during scan...");
    }

    return availableSerialPorts;
}

bool SerialPort::isLocked()
{
    return false;
}

bool SerialPort::setLock()
{
    return false;
}

bool SerialPort::removeLock()
{
    return false;
}

void SerialPort::setLatency(int latency)
{
    if (latency > 0 && latency < 128)
    {
        ttyDeviceLatencyTime = latency;
    }
    else
    {
        TRACE_WARNING(SERIAL, "Invalid latency value: '%i', not in ]0;128[ range.", latency);
    }
}

std::string SerialPort::getDeviceName()
{
    return ttyDeviceName;
}

std::string SerialPort::getDevicePath()
{
    return ttyDevicePath;
}

int SerialPort::getDeviceBaudRate()
{
    return ttyDeviceBaudRate;
}
