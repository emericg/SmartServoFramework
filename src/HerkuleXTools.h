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
 * \file HerkuleXTools.h
 * \date 07/07/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef HERKULEX_TOOLS_H
#define HERKULEX_TOOLS_H

#include "Utils.h"
#include <string>

/*!
 * Refers to total Packe size (in Bytes) from Header to Data. The maximum Packet
 * Size 233, if the packet size is larger than 223 Bytes, packet may not be
 * recognized. Minimum packet size without any data is 7.
 */
#define MAX_PACKET_LENGTH_hkx    (233)

/*!
 * \brief The different status errors available through the r(Status Error) bitfield.
 */
enum {
    ERRBIT_VOLTAGE      = 0x01,
    ERRBIT_ALLOWED_POT  = 0x02,
    ERRBIT_OVERHEAT     = 0x04,
    ERRBIT_INVALID_PKT  = 0x08,
    ERRBIT_OVERLOAD     = 0x10,
    ERRBIT_DRIVER_FAULT = 0x20,
    ERRBIT_EEP_REG_DIST = 0x40
};

/*!
 * \brief The different status details available through the r(Status Detail) bitfield.
 */
enum {
    STATBIT_MOVING          = 0x01,
    STATBIT_INPOSITION      = 0x02,
    STATBIT_CHECKSUM_FLAG   = 0x04,
    STATBIT_UNKWOWN_CMD     = 0x08,
    STATBIT_RANGE           = 0x10,
    STATBIT_GARBAGE         = 0x20,
    STATBIT_TORQUE_ON       = 0x40
};

/*!
 * \brief Get an HerkuleX model name from a model number.
 * \param model_number: The model number of the HerkuleX servo.
 * \return A string containing the textual name for the servo.
 */
std::string hkx_get_model_name(const int model_number);

/*!
 * \brief Get an HerkuleX serie (010x, 020x, 040x, 060x) and model (0101, 0602, ...) from a model number.
 * \param[in] model_number: The model number of the HerkuleX servo.
 * \param[out] servo_serie: A servo serie from ServoDevices_e enum.
 * \param[out] servo_model: A servo model from ServoDevices_e enum.
 */
void hkx_get_model_infos(const int model_number, int &servo_serie, int &servo_model);

/*!
 * \brief Return an HerkuleX model (0101, 0602, ...) from a model number.
 * \param model_number: The model number of an HerkuleX servo.
 * \return A servo model from ServoDevices_e enum.
 */
int hkx_get_servo_model(const int model_number);

/*!
 * \brief Convert an HerkuleX "baudnum" to a baudrate in bps.
 * \param baudnum: HerkuleX "baudnum", from 0 to 254.
 * \param servo_serie: The servo serie, used to compute adequate baudrate.
 * \return A valid baudrate, from 57600 to 1000000 bps.
 */
int hkx_get_baudrate(const int baudnum, const int servo_serie = SERVO_DRS);

#endif /* HERKULEX_TOOLS_H */
