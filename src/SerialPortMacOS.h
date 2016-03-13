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
 * \date 14/07/2015
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef SERIALPORT_MACOS_H
#define SERIALPORT_MACOS_H

#if defined(__APPLE__) || defined(__MACH__)

#include "SerialPort.h"

/*!
 * \brief The serial ports scanner function.
 * \param[out] availableSerialPorts: A list of serial port nodes (ex: /dev/cu.USB0 or /dev/cu.modem1234).
 * \return The number of serial ports found.
 *
 * Scans for devices identifying as modems, using IOKit. This feature is still a work in progress.
 */
int serialPortsScanner(std::vector <std::string> &availableSerialPorts);

/*!
 * \brief The SerialPortMacOS class, based on the Linux version.
 * \todo The MacOS port is widely untested.
 * \todo Reimplement the auto-detection methods from the SerialPortLinux class.
 */
class SerialPortMacOS: public SerialPort
{
    int ttyDeviceFileDescriptor;   //!< The file descriptor that will be used to write to the serial device.
    int ttyDeviceBaudRateFlag;     //!< Speed of the serial device, from a <termios.h> enum.
    bool ttyCustomSpeed;           //!< Try to set custom speed on the serial port.

    /*!
     * \brief Get current time since the Epoch.
     * \return Current time since the Epoch in milliseconds.
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

    /*!
     * \brief SerialPortLinux::convertBaudRateFlag.
     * \param baudrate: baudrate in baud.
     * \return A baudRateFlag representing the target speed of the serial device, from a <termios.h> enum.
     *
     * This function will try to match a baudrate with an existing baudrate flag,
     * or at least one +/- 1.5% close. When this is not possible, the 'ttyCustomSpeed'
     * flag is set, and the openLink() function will try to set a custom speed.
     */
    int convertBaudRateFlag(int baudrate);

    /*!
     * \brief Check if the serial device has been locked by another instance or program.
     * \return True if a lock has been found for this serial device, false otherwise.
     */
    bool isLocked();

    /*!
     * \brief Set a lock for this serial device.
     * \return True if a lock has been placed successfully for this serial device, false otherwise.
     *
     * We have several ways of doing that:
     * - Use a file lock in "/tmp" directory. Will only work accross SmartServoFramework instances.
     * - Use TIOCEXCL ioctl.
     */
    bool setLock();

    /*!
     * \brief Remove the lock we put on this serial device.
     * \return True if the lock has been removed successfully for this serial device, false otherwise.
     */
    bool removeLock();

public:
    SerialPortMacOS(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN, const int servoDevices = SERVO_UNKNOWN);
    ~SerialPortMacOS();

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

#endif /* defined(__APPLE__) || defined(__MACH__) */

#endif /* SERIALPORT_MACOS_H */
