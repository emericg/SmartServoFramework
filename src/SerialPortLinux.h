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
 * \file SerialPortLinux.h
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERIALPORT_LINUX_H
#define SERIALPORT_LINUX_H

#if defined(__linux__) || defined(__gnu_linux)

#include "SerialPort.h"

/*!
 * \brief The serial ports scanner function.
 * \param[out] availableSerialPorts: A list of serial port nodes (ex: /dev/ttyUSB0).
 * \return The number of serial ports found.
 *
 * Scans for /dev/ttyUSB* and /dev/ttyACM* ports. Regular serial devices on
 * /dev/ttyS* are not scanned as they are always considered as valid even with
 * no USB2Dynamixel / USB2AX or TTL adapter attached.
 */
int serialPortsScanner(std::vector <std::string> &availableSerialPorts);

/*!
 * \brief The SerialPortLinux class.
 */
class SerialPortLinux: public SerialPort
{
    int ttyDeviceFileDescriptor;   //!< The file descriptor that will be used to write to the serial device.
    int ttyDeviceBaudRateFlag;     //!< Speed of the serial device, from a <termios.h> enum.
    bool ttyCustomSpeed;           //!< Try to set custom speed on the serial port.
    bool ttyLowLatency;            //!< Try to set low latency flag on the serial port (works only on FTDI based adapters).

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
     * \brief Check if the serial link has been "file locked" by another instance.
     * \return True is a file lock has been found for this serial link, false otherwise.
     *
     * Use /tmp directory for lock storage.
     */
    bool isLocked();

public:
    /*!
     * \brief SerialPortLinux constructor will only init some variables to default values.
     * \param devicePath: The path to the serial device (ex: /dev/ttyUSB0") (cannot be changed after the constructor).
     * \param baud: Can be a 'baudrate' (in bps) or a Dynamixel / HerkuleX 'baudnum'.
     * \param serialDevice: Specify (if known) what TTL converter is in use.
     * \param servoDevices: Specify if we use this serial port with Dynamixel or HerkuleX devices.
     *
     * \note: devicePath can be set to "auto", serial port autodetection will be
     * triggered, and the first serial port available will be used.
     */
    SerialPortLinux(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN, const int servoDevices = SERVO_UNKNOWN);
    ~SerialPortLinux();

    int openLink();
    bool isOpen();
    void closeLink();

    int tx(unsigned char *packet, int packetLength);
    int rx(unsigned char *packet, int packetLength);
    void flush();

    /*!
     * \brief switchHighSpeed() should enable ASYNC_LOW_LATENCY flag and reduce latency_timer per tty device (need root credential)
     * \return True in case of success.
     * \todo This function is not implemented yet.
     */
    bool switchHighSpeed();

    /*!
     * \brief Set a "file lock" for this serial link.
     * \return True is a file lock has been created successfully this serial link, false otherwise.
     * \todo latency_timer value auto-detection doesn't produce intended result yet.
     *
     * Use /tmp directory for lock storage.
     */
    bool setLock();

    void setLatency(int latency);
    void setTimeOut(int packetLength);
    void setTimeOut(double msec);
    int checkTimeOut();
};

#endif /* __linux__ || __gnu_linux */

#endif /* SERIALPORT_LINUX_H */
