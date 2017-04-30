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
 * \file ServoX.cpp
 * \date 29/04/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoX.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"

#include "minitraces.h"

ServoX::ServoX(int dynamixel_id, int dynamixel_model, int control_mode):
    ServoDynamixel(XMXH_control_table, dynamixel_id, dynamixel_model, control_mode)
{
    runningDegrees = 360;
    steps = 4096;
}

ServoX::~ServoX()
{
    //
}

/* ************************************************************************** */

int ServoX::getAlarmLed()
{
    return 0; // Not available on X series
}

void ServoX::setAlarmLed()
{
    return; // Not available on X series
}

int ServoX::getLock()
{
    return 0; // Not available on X series
}

void ServoX::setLock()
{
    return; // Not available on X series
}

/* ************************************************************************** */

int ServoX::getControlMode()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CONTROL_MODE)];
}

int ServoX::getDGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_D_GAIN)];
}

int ServoX::getIGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_I_GAIN)];
}

int ServoX::getPGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_P_GAIN)];
}

int ServoX::getHardwareErrorStatus()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_HW_ERROR_STATUS)];
}

/* ************************************************************************** */

void ServoX::setId(int id)
{
    TRACE_1(DXL, "[#%i] setId(from %i to %i)", servoId, servoId, id);

    if (id > -1 && id < 253)
    {
        std::lock_guard <std::mutex> lock(access);

        // Maybe check if new ID is not already in use ?
        registerTableValues[gid(REG_ID)] = id;
        registerTableCommits[gid(REG_ID)] = 1;
    }
}

void ServoX::setError(const int error)
{
    // On protocol v2, the error is not a bitfield but a simple error number,
    // so we cannot keep adding errors fields

    if (error != 0)
    {
        std::lock_guard <std::mutex> lock(access);
        statusError = error;
    }
}
