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
 * \file DynamixelController.h
 * \date 14/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include "ControllerAPI.h"
#include "Dynamixel.h"

#include "ServoDynamixel.h"
#include "ServoAX.h"
#include "ServoEX.h"
#include "ServoMX.h"
#include "ServoXL.h"

#include <vector>
#include <mutex>
#include <thread>

/** \addtogroup ControllerAPIs
 *  @{
 */

/*!
 * \brief The DynamixelController class
 *
 * A "controller" provide a high level API to handle several servos at the same time.
 * A client must instanciate servos objects and register them to a controller.
 * Each servo object is synchronized with its hardware counterpart by the run()
 * method, running in its own backgound thread.
 *
 * A DynamixelController instance can only be attached to ONE serial link at a time.
 */
class DynamixelController: public Dynamixel, public ControllerAPI
{
    //! Compute some internal settings (ackPolicy, maxId, protocolVersion) depending on current servo serie and serial device.
    void updateInternalSettings();

    //! Read/write synchronization loop, running inside its own background thread
    void run();

public:
    /*!
     * \brief DynamixelController constructor.
     * \param freq: This is the synchronization frequency between the controller and the servos devices. Range is [1;120], default is 30.
     * \param servoSerie: The servo class to use with this instance (default is Dynamixel MX, the more 'capable' of the Dynamixel v1 serie).
     */
    DynamixelController(int freq = 30, int servoSerie = SERVO_MX);

    /*!
     * \brief DynamixelController destructor. Stop the controller's thread and close the serial connection.
     */
    ~DynamixelController();

    /*!
     * \brief Change communication protocol version for this controller instance.
     * \param protocol: The Dynamixel communication protocol to use. Can be v1 or v2.
     */
    void changeProtocolVersion(int protocol);

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
     * \brief Scan a serial link for Dynamixel devices.
     * \param start: First ID to be scanned.
     * \param stop: Last ID to be scanned.
     *
     * Note: Be aware that calling this function will reset the current servoList.
     *
     * This scanning function will ping every Dynamixel ID (from 'start' to 'stop',
     * default [0;253]) on a serial link, and use the status response to detect
     * the presence of a device.
     * When a device is being scanned, its LED is briefly switched on.
     * Every servo found will be automatically registered to this controller.
     *
     * The current value 'protocolVersion' will be used. You can change it with
     * the setProtocolVersion() function before calling autodetect().
     */
    void autodetect_internal(int start = 0, int stop = 253);

    // Wrappers
    std::string serialGetCurrentDevice_wrapper();
    std::vector <std::string> serialGetAvailableDevices_wrapper();
    void serialSetLatency_wrapper(int latency);
    void serialLockInterface_wrapper();
};

/** @}*/

#endif /* DYNAMIXEL_CONTROLLER_H */
