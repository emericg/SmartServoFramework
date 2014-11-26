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
 * \file SerialPortWindows.h
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERIALPORT_WINDOWS_H
#define SERIALPORT_WINDOWS_H

#if defined(_WIN32) || defined(_WIN64)

#include "SerialPort.h"

// Windows specific
#define NOMINMAX // Not sure if really needed when building the framework without Qt, but still...
#undef UNICODE // Avoid runtime errors when building with MSVC...

#include <windows.h>

/*!
 * \brief Convert regular std::string into 'LPCWSTR' Windows unicode string format.
 */
LPCWSTR stringToLPCWSTR(const std::string &s);

/*!
 * \brief The serial ports scanner function.
 * \param[out] availableSerialPorts: A list of serial port nodes (ex: "\\.\COM1").
 * \return The number of serial ports found.
 *
 * Perform a "reverse" scan, from high port numbers to low numbers, as high numbers
 * are used most of the time by other virtual devices.
 */
int serialPortsScanner(std::vector <std::string> &availableSerialPorts);

/*!
 * \brief The SerialPortWindows class
 */
class SerialPortWindows: public SerialPort
{
    HANDLE ttyDeviceFileDescriptor; //!< The file descriptor that will be used to write to the serial device.

    /*!
     * \brief Retrieves the current value of the performance counter, which is a high resolution (<1us) time stamp that can be used for time-interval measurements.
     * \return A high resolution time stamp (unit?).
     */
    double getTime();

    /*!
     * \brief Set baudrate for this interface.
     * \param baud: Can be a 'baudrate' (in bps) or a Dynamixel / HerkuleX 'baudnum'.
     *
     * Must be called before openLink(), otherwise it will have no effect until the
     * next connection.
     */
    void setBaudRate(const int baud);

public:
    /*!
     * \brief SerialPortWindows constructor will only init some variables to default values.
     * \param devicePath: The path to the serial device (ex: "\\\\.\\COM1") (the string needs to be escaped) (cannot be changed after the constructor).
     * \param baud: Can be a 'baudrate' (in bps) or a Dynamixel / HerkuleX 'baudnum'.
     * \param serialDevice: Specify (if known) what TTL converter is in use.
     * \param servoDevices: Specify if we use this serial port with Dynamixel or HerkuleX devices.
     *
     * \note: devicePath can be set to "auto", serial port autodetection will be
     * triggered, and the first serial port available will be used.
     */
    SerialPortWindows(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN, const int servoDevices = SERVO_UNKNOWN);
    ~SerialPortWindows();

    int openLink();
    bool isOpen();
    void closeLink();

    int tx(unsigned char *packet, int packetLength);
    int rx(unsigned char *packet, int packetLength);
    void flush();

    void setTimeOut(int packetLength);
    void setTimeOut(double msec);
    int checkTimeOut();
};

#endif /* defined(_WIN32) || defined(_WIN64) */

#endif /* SERIALPORT_WINDOWS_H */
