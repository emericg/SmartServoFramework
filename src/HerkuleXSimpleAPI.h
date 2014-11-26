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
 * \file HerkuleXSimpleAPI.h
 * \date 19/08/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef HERKULEX_SIMPLE_API_H
#define HERKULEX_SIMPLE_API_H

#include "HerkuleX.h"

#include <vector>
#include <mutex>

/** \addtogroup SimpleAPIs
 *  @{
 */

/*!
 * \brief The HerkuleXSimpleAPI class
 *
 * This class provide a high level API to handle servos with minimal efforts.
 * One "Simple API" instance can only be attached to ONE serial link at a time,
 * so you can create as many instances as you need on different ports.
 *
 * You must specify what device class you will be using for this API to operate
 * efficiently. If you want to use this API with multiple servo series at the
 * same time, just choose the more permissive serie. 'SERVO_DRS' should be
 * fine for almost every use cases.
 *
 * IMPORTANT NOTE:
 * HerkuleX devices operate differently than Dynamixel devices. A number of available
 * registers are present in both ROM and RAM memory. ROM values are copied to RAM
 * when a servo is powered-on.
 * - When reading these register with helpers from this API, the RAM value will be returned by default.
 * - When writing, both ROM and RAM values will be written (if possible).
 * If you need a finer grained control, you can use the getSetting and SetSetting
 * functions that will allow you to choose ROM or RAM targets.
 *
 * The "getter" functions return the wanted value or '-1' if an error occurs.
 * The "setter" functions return '0' if an error occurs, '1' otherwise.
 */
class HerkuleXSimpleAPI: public HerkuleX
{
    const int (*ct)[8];     //!< Pointer to the control table for a given servo class (selected by the constructor).

    /*!
     * \brief Validate an ID.
     * \param id: The servo ID we need to validate.
     * \param broadcast: Indicate if the 'Broadcast ID' is allowed (default is 'true').
     * \return true if the ID is valid, false otherwise.
     */
    bool checkId(const int id, const bool broadcast = true);

public:
    /*!
     * \brief HerkuleXSimpleAPI constructor.
     * \param servos: The servo class to use with this instance.
     */
    HerkuleXSimpleAPI(int servos = SERVO_DRS);

    /*!
     * \brief HerkuleXSimpleAPI destructor.
     *
     * The destructor calls the disconnect() function to make sure the serial
     * port is properly freed.
     */
    ~HerkuleXSimpleAPI();

    /*!
     * \brief Connect the SimpleAPI to a serial port.
     * \param devicePath: The serial port device node.
     * \param baud: The serial port speed, can be a baud rate or a 'baudnum'.
     * \param serialDevice: If known, the serial adapter model used by this link.
     * \return 1 if the connection is successfull, 0 otherwise.
     */
    int connect(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN);

    /*!
     * \brief Stop the serial connection.
     */
    void disconnect();

    /*!
     * \brief Scan a serial link for HerkuleX devices.
     * \param start: First ID to be scanned.
     * \param stop: Last ID to be scanned.
     * \return A vector of HerkuleX IDs found during the scan.
     *
     * This scanning function will ping every HerkuleX ID (from 'start' to 'stop',
     * default [0;253]) on a serial link, and use the status response to detect
     * the presence of a device.
     * When a device is being scanned, its LED is briefly switched on.
     */
    std::vector <int> servoScan(int start = 0, int stop = 253);

    /*!
     * \brief Send "ping" command to a servo.
     * \param id: The servo to ping.
     * \param status: A pointer to a PingResponse structure, containing model_number and firmware version infos.
     */
    bool ping(const int id, PingResponse *status);

    /*!
     * \brief Reboot a servo.
     * \param id: The servo to reboot.
     *
     * Be careful when rebooting a device, do not try to use it righ afer sending
     * a reboot command, it will be unavailable for a short periode of time.
     */
    void reboot(const int id);

    /*!
     * \brief Reset servo to factory settings.
     * \param id: The servo to reset.
     * \param setting: You can control what to erase using the '::ResetOptions_e' enum.
     */
    void reset(const int id, const int setting);

    /*!
     * \brief Read the model number of a given device.
     * \param id: Device ID.
     * \return The model number.
     */
    int readModelNumber(const int id);

    /*!
     * \brief Read the firmware version of a given device.
     * \param id: Device ID.
     * \return The firmware version.
     */
    int readFirmwareVersion(const int id);

    int changeId(const int old_id, const int new_id);
    int changeBaudRate(const int id, const int baudnum);

    void getMinMaxPositions(const int id, int &min, int &max);
    int setMinMaxPositions(const int id, const int min, const int max);

    int getTorqueEnabled(const int id);
    int setTorqueEnabled(const int id, int torque);

    int getLed(const int id);
    int setLed(const int id, int led, const int color = LED_RED);

    int turn(const int id, const int velocity);

    int getGoalPosition(const int id);
    int setGoalPosition(const int id, const int position);
    int setGoalPosition(const int id, const int position, const int speed);
    int getGoalSpeed(const int id);
    int setGoalSpeed(const int id, const int speed);

    int readCurrentPosition(const int id);
    int readCurrentSpeed(const int id);
    int readCurrentLoad(const int id);
    double readCurrentVoltage(const int id);
    double readCurrentTemperature(const int id);

    // General purpose getters/setters // FIXME reg_addr different if RAM or ROM register
    int getSetting(const int id, const int reg_name, int reg_type = REGISTER_BOTH, const int device = SERVO_HERKULEX);
    int setSetting(const int id, const int reg_name, const int reg_value, int reg_type = REGISTER_BOTH, const int device = SERVO_HERKULEX);
};

/** @}*/

#endif /* HERKULEX_SIMPLE_API_H */
