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
 * \file SerialPortMacOS.h
 * \date 13/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERIALPORT_MACOS_H
#define SERIALPORT_MACOS_H

#if defined(__APPLE__) || defined(__MACH__)

#include "SerialPort.h"

/*!
 * \brief The serial ports scanner function.
 * \param[out] availableSerialPorts: A list of serial port nodes (ex: /dev/tty.USB0 or /dev/cu.USB0).
 * \return The number of serial ports found.
 *
 * Scans for /dev/ttyUSB* and /dev/ttyACM* ports. Regular serial devices on
 * /dev/ttyS* are not scanned as they are always considered as valid even with
 * no USB2Dynamixel / USB2AX or regular TTL converter attached.
 *
 * "ls /dev/cu.*"
 */
int serialPortsScanner(std::vector <std::string> &availableSerialPorts);

/*!
 * \brief The SerialPortMacOS class, based on the Linux version.
 * \todo The MacOS port is widely untested.
 * \todo Reimplement the auto-detection methods from the SerialPortLinux class.
 */
class SerialPortMacOS: public SerialPortLinux
{
    SerialPortMacOS(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN, const int servoDevices = SERVO_UNKNOWN);
    ~SerialPortMacOS();

    // TODO

    void setLatency(int latency);
};

#endif /* defined(__APPLE__) || defined(__MACH__) */

#endif /* SERIALPORT_MACOS_H */
