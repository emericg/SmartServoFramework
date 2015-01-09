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
 * \file SerialPort.h
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include "Utils.h"
#include <string>
#include <vector>

/*!
 * \brief Latency time (in milliseconds) on the serial port.
 *
 * This timer should be carefully choosed depending on your OS and the speed of
 * your serial port implementation. Default is set to a high value to avoid any
 * problem.
 */
#define LATENCY_TIME_DEFAULT    (48)

/*!
 * \brief Specify which serial device chip we are using.
 *
 * This information will be used to access some extra features like SYNC_READ
 * for Dynamixels devices, and check maximum baudrate/latency available.
 */
enum SerialDevices_e
{
    SERIAL_UNKNOWN       = 0,

    SERIAL_USB2DYNAMIXEL = 1,
    SERIAL_USB2AX        = 2,
    SERIAL_ZIG100        = 3,   //!< ZIG-100 / 110A
    SERIAL_BT100         = 4,   //!< BT-100 / 110A
    SERIAL_BT210         = 5,

    SERIAL_OTHER_FTDI    = 10,  //!< Devices based on FTDI chips
    SERIAL_OTHER_CP210x  = 11,  //!< Devices based on CP210x chips
};

/*!
 * \brief The different return status code available for serial packet communication.
 */
enum SerialErrorCodes_e
{
    COMM_TXSUCCESS = 0,                     //!< Instruction packet was sent successfully
    COMM_RXSUCCESS,                         //!< Status packet was received successfully

    COMM_TXFAIL,                            //!< Error when sending instruction packet
    COMM_RXFAIL,                            //!< Error when receiving status packet
    COMM_TXERROR,                           //!< Invalid instruction packet, nothing was sent
    COMM_RXWAITING,                         //!< Waiting for a status packet
    COMM_RXTIMEOUT,                         //!< Timeout reached while waiting for a status packet
    COMM_RXCORRUPT                          //!< Status packet corrupted
};

/*!
 * \brief The SerialPort base class.
 *
 * This class provide abstraction to use the serial port across Linux, Windows
 * and Mac OS operating systems. Almost all of this code is heavily OS dependent,
 * and therefore most functions are "virtual pure" and implemented only in child
 * classes.
 *
 * Both Dynamixel and HerkuleX devices are using the same settings:
 * - Data Bit: 8
 * - Stop Bit: 1
 * - No parity
 * - No flow control
 *
 * We support baudrate values from:
 * - 9.6k to 10.5M for Dynamixels.
 * - 57.6k to 1M for HerkuleX.
 *
 * Note: we will assume 1 baud = 1 bit when using modern UART implementations
 * and TTL converters which only transfer one symbole at a time.
 * More details: http://en.wikipedia.org/wiki/Gross_bit_rate#Gross_bit_rate
 */
class SerialPort
{
protected:
    std::string ttyDeviceName;     //!< The name of the serial device computed from ttyDevicePath (ex: "ttyUSB0" or "COM1").
    std::string ttyDevicePath;     //!< The path to the serial device (ex: "/dev/ttyUSB0" or "\\.\COM1").
    int ttyDeviceBaudRate;         //!< Speed of the serial link in baud. Default is 1M/s.
    int ttyDeviceLatencyTime;      //!< The value of this timer (in millisecond) should be carefully choosed depending on your OS and the speed of your serial port implementation.

    std::string ttyDeviceLockPath; //!< The path to a "lock file" to lock serial interface against concurrent use by multiple programs.
    bool ttyDeviceLocked;          //!< Set to true if a "lock file" has been set by this SerialPort instance.

    int serialDevice;              //!< Specify (if known) what TTL converter is in use. This information will be used to compute correct baudrate.
    int servoDevices;              //!< Specify if we use this serial port with Dynamixel or HerkuleX devices (using ::ServoDevices_e values). This information will be used to compute correct baudrate.

    double packetStartTime;        //!< Time (in millisecond) where the packet was sent.
    double packetWaitTime;         //!< Time (in millisecond) to wait for an answer.
    double byteTransfertTime;      //!< Estimation of the time (in millisecond) needed to read/write one byte on the serial link.

    /*!
     * \brief Get the current time.
     * \return The current time in milliseconds.
     */
    virtual double getTime() = 0;

    /*!
     * \brief Set baudrate for this interface.
     * \param baud: Can be a 'baudrate' (in bps) or a Dynamixel / HerkuleX 'baudnum'.
     *
     * Must be called before openLink(), otherwise it will have no effect until the
     * next connection.
     */
    virtual void setBaudRate(const int baud) = 0;

    /*!
     * \brief Check and convert (if needed) a Dynamixel / HerkuleX 'baudnum' into a regular 'baudrate' with a plausible value.
     * \param baud: Can be a 'baudrate' in bps or a Dynamixel / HerkuleX 'baudnum'.
     * \return A valid 'baudrate' value in baud.
     *
     * - If the input value is inferior to 255, we have a valid 'baudnum' and will use a conversion depending on the current 'servoSerie' value.
     * - If its  superior or equal to 2400, we consider it to be a valid baudrate and will use it as it is.
     * - In case of an obviously wrong input value or failed conversion, we will try default the fallback speed
     *   depending on the current 'servoSerie' value.
     *
     * If the 'serialDevice' value is set, this function will also check the baudrate
     * against maximum bandwith available depending on the serial device used (USB2Dynamixel, USB2AX, ...).
     */
    int checkBaudRate(const int baud);

    /*!
     * \brief Check if the serial link has been "file locked" by another instance.
     * \return True is a file lock has been found for this serial link, false otherwise.
     *
     * \note This functionnality is only implemented on the Linux backend.
     */
    virtual bool isLocked();

public:
    /*!
     * \brief SerialPort constructor will only init some variables to default values.
     * \param serialDevice: Specify (if known) what TTL converter is in use (using ::SerialDevices_e).
     * \param servoDevices: Specify if we use this serial port with Dynamixel or HerkuleX devices (using ::ServoDevices_e).
     */
    SerialPort(const int serialDevice, const int servoDevices);

    /*!
     * \brief SerialPort destructor makes sure the serial link is closed.
     */
    virtual ~SerialPort();

    /*!
     * \brief Autoselect the first available serial port.
     * \return The string corresponding to the port path autodetected, or "null" if none available.
     */
    std::string autoselectSerialPort();

    /*!
     * \brief Scan available serial port(s) to find available devices.
     * \return A vector of available port names.
     */
    std::vector <std::string> scanSerialPorts();

    /*!
     * \brief Open a serial link at given speed.
     * \return 1 if success, 0 otherwise.
     */
    virtual int openLink() = 0;

    /*!
     * \brief Check if the serial link is open.
     * \return true if the serial link is open.
     */
    virtual bool isOpen() = 0;

    /*!
     * \brief Flush incoming datas and close the serial link file handle.
     */
    virtual void closeLink() = 0;

    /*!
     * \brief Send a packet over a serial link.
     * \param[in] packet: Data packet to send.
     * \param packetLength: Size in byte(s) of data packet to transmit.
     * \return Size in byte(s) sent to the serial link.
     */
    virtual int tx(unsigned char *packet, int packetLength) = 0;

    /*!
     * \brief Receive a packet over a serial link.
     * \param[out] packet: Data packet received.
     * \param packetLength: Size in byte(s) of data packet received.
     * \return Size in byte(s) received from the serial link.
     */
    virtual int rx(unsigned char *packet, int packetLength) = 0;

    /*!
     * \brief Flush non-read input data.
     */
    virtual void flush() = 0;

    /*!
     * \brief Set a "file lock" for this serial link.
     * \return True is a file lock has been created successfully this serial link, false otherwise.
     *
     * \note This functionnality is only implemented on the Linux backend.
     */
    virtual bool setLock();

    /*!
     * \brief Set the serial port latency for packet reception timeout.
     * \param latency: The serial port latency in millisecond.
     */
    virtual void setLatency(int latency);

    /*!
     * \brief Set the maximum duration to wait for an answer, computed from packetLength and latencyTime.
     * \param packetLength: Number of byte to received, will be used to compute the duration of the timeout.
     */
    virtual void setTimeOut(int packetLength) = 0;

    /*!
     * \brief Set the maximum duration to wait for an answer.
     * \param msec: Duration of the timeout in millisecond.
     */
    virtual void setTimeOut(double msec) = 0;

    /*!
     * \brief Check if the response has timeouted.
     * \return 1 if timeout, 0 if we still need to wait.
     */
    virtual int checkTimeOut() = 0;

    /*!
     * \brief Get serial device name.
     * \return A string containing the device name.
     */
    std::string getDeviceName();

    /*!
     * \brief Get serial device path currently is use.
     * \return A string containing the device path.
     */
    std::string getDevicePath();

    /*!
     * \brief Get serial device baudrate currently is use.
     * \return An integer containing the device baudrate.
     */
    int getDeviceBaudRate();
};

#endif /* SERIALPORT_H */
