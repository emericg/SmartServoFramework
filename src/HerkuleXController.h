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
 * \file HerkuleXController.h
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef HERKULEX_CONTROLLER_H
#define HERKULEX_CONTROLLER_H

#include "ControllerAPI.h"
#include "HerkuleX.h"

#include "ServoHerkuleX.h"
#include "ServoDRS.h"

#include <vector>

/** \addtogroup ManagedAPIs
 *  @{
 */

/*!
 * \brief The HerkuleXController class, part of the ManagedAPI
 *
 * An controller can only be attached to ONE serial link at a time.
 */
class HerkuleXController: public HerkuleX, public ControllerAPI
{
    //! Compute some internal settings (ackPolicy, maxId, protocolVersion) depending on current servo serie and serial device.
    void updateInternalSettings();

    //! Read/write synchronization loop, running inside its own background thread
    void run();

public:
    /*!
     * \brief HerkuleXController constructor.
     * \param servoSerie: The servo serie to use with this controller. Only used to choose the right communication protocol.
     * \param ctrlFrequency: This is the synchronization frequency between the controller and the servos devices. Range is [1;120], default is 30.
     */
    HerkuleXController(int ctrlFrequency = 30, int servoSerie = SERVO_DRS);

    /*!
     * \brief HerkuleXController destructor. Stop the controller's thread and close the serial connection.
     */
    ~HerkuleXController();

    /*!
     * \brief Connect the controller to a serial port, if the connection is successfull start a synchronization thread.
     * \param devicePath: The serial port device node.
     * \param baud: The serial port speed, can be a baud rate or a 'baudnum'.
     * \param serialDevice: If known, the serial adapter model used by this link.
     * \return 1 if the connection is successfull, 0 otherwise.
     */
    int connect(std::string &devicePath, const int baud, const int serialDevice = SERIAL_UNKNOWN);

    /*!
     * \brief Stop the controller's thread, clean umessage queue, and close the serial connection.
     */
    void disconnect();

    /*!
     * \brief Scan a serial link for HerkuleX devices.
     * \param start: First ID to be scanned.
     * \param stop: Last ID to be scanned.
     *
     * Note: Be aware that calling this function will reset the current servoList.
     *
     * This scanning function will ping every HerkuleX ID (from 'start' to 'stop',
     * default [0;253]) on a serial link, and use the status response to detect
     * the presence of a device.
     * When a device is being scanned, its LED is briefly switched on.
     * Every servo found will be automatically registered to this controller.
     */
    void autodetect_internal(int start = 0, int stop = 253);

    // Wrappers
    std::string serialGetCurrentDevice_wrapper();
    std::vector <std::string> serialGetAvailableDevices_wrapper();
    void serialSetLatency_wrapper(int latency);
};

/** @}*/

#endif // HERKULEX_CONTROLLER_H
