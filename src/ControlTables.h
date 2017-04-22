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
 * \file ControlTables.h
 * \date 21/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef CONTROL_TABLES_H
#define CONTROL_TABLES_H
/* ************************************************************************** */

/** \addtogroup ControlTables
 *  @{
 */

// "Control Table consists of data regarding the current status and operation,
// which exists inside of Dynamixel. The user can control Dynamixel by changing
// data of Control Table via Instruction Packet".

// Control Table legend:
// Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value
// "-1" indicate a 'not applicable' field.
// "-2" indicate a 'servo model specific' field.

enum RegisterAddrType_e
{
    REGISTER_AUTO   = 0,    //!< Return the ROM if available, RAM otherwise
    REGISTER_ROM    = 1,
    REGISTER_RAM    = 2,
    REGISTER_BOTH   = 3,
};

/*!
 * \brief The RegisterAccessMode enum indicates if a servo register can be read and/or write.
 */
enum RegisterAccessMode_e
{
    READ_ONLY  = 0,
    READ_WRITE = 1
};

/*!
 * \brief RegisterInfos structure
 */
typedef struct RegisterInfos
{
    int reg_index;       //!< Register index in device's control table
    int reg_addr;        //!< Register address in RAM if available, ROM overwise

    int reg_addr_rom;    //!< Register address in ROM (if available)
    int reg_addr_ram;    //!< Register address in RAM (if available)
    int reg_size;        //!< Register size in byte
    int reg_access_mode; //!< Register access mode (read/write or read only)
    int reg_value_def;   //!< Default (or initial) value
    int reg_value_min;   //!< Minimum value
    int reg_value_max;   //!< Maximum value

} RegisterInfos;

/* ************************************************************************** */

/*!
 * \brief Get a control table for a given servo device.
 * \param servo_model: A servo model from ServoDevices_e enum, NOT a model number extracted from a device!
 * \return A control table corresponding to the given device.
 */
const int (*getRegisterTable(const int servo_model))[8];

/*!
 * \brief Get a control table for a given servo device.
 * \param servo_serie: A servo serie from ServoDevices_e enum.
 * \param servo_model: A servo model from ServoDevices_e enum, NOT a model number extracted from a device!
 * \return A control table corresponding to the given device.
 */
const int (*getRegisterTable(const int servo_serie, const int servo_model))[8];

/*!
 * \brief Get a control table for a given servo device.
 * \param ct: A device's control table.
 * \return The number of register into the given control table.
 */
unsigned getRegisterCount(const int ct[][8]);

/* ************************************************************************** */

int getRegisterInfos(const int ct[][8], const int reg_name, RegisterInfos &infos);

int getRegisterTableIndex(const int ct[][8], const int reg_name);

int getRegisterName(const int ct[][8], const int reg_index);

int getRegisterAddr(const int ct[][8], const int reg_name, const int reg_type = REGISTER_AUTO);

int getRegisterSize(const int ct[][8], const int reg_name);

int getRegisterAccessMode(const int ct[][8], const int reg_name);

int getRegisterInitialValue(const int ct[][8], const int reg_name);

int getRegisterBounds(const int ct[][8], const int reg_name, int &min, int &max);

/** @}*/

/* ************************************************************************** */
#endif // CONTROL_TABLES_H
