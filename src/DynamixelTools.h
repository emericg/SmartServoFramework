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
 * \file DynamixelTools.h
 * \date 11/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef DYNAMIXEL_TOOLS_H
#define DYNAMIXEL_TOOLS_H

#include "Utils.h"
#include <string>

/*!
 * \brief Max packet size with Dynamixel communication protocol v1.
 * The '150' bytes size limit seems to be arbitrary, and the real limit might
 * be depending on the RX buffer size of particular servo models.
 */
#define MAX_PACKET_LENGTH_dxlv1    (150)

/*!
 * \brief Max packet size with Dynamixel communication protocol v2.
 */
#define MAX_PACKET_LENGTH_dxlv2    (65535)

/*!
 * \brief The different errors available through the error bitfield for Dynamixel protocol v1.
 */
enum {
    ERRBIT1_VOLTAGE      = 0x01,
    ERRBIT1_ANGLE_LIMIT  = 0x02,
    ERRBIT1_OVERHEAT     = 0x04,
    ERRBIT1_RANGE        = 0x08,
    ERRBIT1_CHECKSUM     = 0x10,
    ERRBIT1_OVERLOAD     = 0x20,
    ERRBIT1_INSTRUCTION  = 0x40
};

/*!
 * \brief The different errors available through the error number for Dynamixel protocol v2.
 */
enum {
    ERRBIT2_RESULT      = 0x01,
    ERRBIT2_INSTRUCTION = 0x02,
    ERRBIT2_CHECKSUM    = 0x03,
    ERRBIT2_DATA_RANGE  = 0x04,
    ERRBIT2_DATA_LENGTH = 0x05,
    ERRBIT2_DATA_LIMIT  = 0x06,
    ERRBIT2_ACCESS      = 0x07
};

/*!
 * \brief Get a Dynamixel model name from a model number.
 * \param model: The model number of the Dynamixel servo or sensor.
 * \return A string containing the textual name for the device.
 *
 * This function does not handle PRO serie yet.
 */
std::string dxl_get_model_name(const int model);

/*!
 * \brief Get a Dynamixel serie (AX, EX, MX, ...) and model (AX-12A, MX-106, ...) from a model number.
 * \param[in] model_number: The model number of the Dynamixel servo or sensor.
 * \param[out] servo_serie: A servo serie from ServoDevices_e enum.
 * \param[out] servo_model: A servo model from ServoDevices_e enum.
 *
 * This function does not handle PRO serie yet.
 */
void dxl_get_model_infos(const int model_number, int &servo_serie, int &servo_model);

/*!
 * \brief Return a Dynamixel model (AX, EX, MX, ...) from a model number.
 * \param model_number: The model number of a Dynamixel servo.
 * \return A servo model from ServoDevices_e enum.
 */
int dxl_get_servo_model(const int model_number);

/*!
 * \brief Convert a Dynamixel "baudnum" to a baudrate in bps.
 * \param baudnum: Dynamixel "baudnum", from 0 to 254.
 * \param servo_serie: The servo serie, used to compute adequate baudrate.
 * \return A valid baudrate, from 2400 to 10500000 bps.
 *
 * The baudrate is usually computed from baudnum using the following formula:
 * Speed(baudrate) = 2000000/(baudnum+1).
 * Valid baudnum values are in range 1 to 254, which gives us baudrate values
 * of 1MB/s to 7,84kB/s.
 *
 * However, the MX Dynamixel series have a maximum speed of 4.5MB/s (which is
 * the maximum speed for most high-end desktop UART and also for the USB2Dynamixel).
 * To handle MX series correctly and set them to speeds higher than 1MB/s, we use
 * the forumla for the baudnum in range 1-249 and interpret values 250 to 254
 * differently.
 *
 * Dynamixel XL-320 and PRO series use different calculations, and PRO series speeds
 * can go as high as 10.5MB/s!
 */
int dxl_get_baudrate(const int baudnum, const int servo_serie = SERVO_AX);

#endif /* DYNAMIXEL_TOOLS_H */
