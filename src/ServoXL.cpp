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
 * \file ServoXL.cpp
 * \date 08/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoXL.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"

ServoXL::ServoXL(int dynamixel_id, int dynamixel_model, int control_mode):
    ServoDynamixel(XL320_control_table, dynamixel_id, dynamixel_model, control_mode)
{
    runningDegrees = 300;
    steps = 1024;
}

ServoXL::~ServoXL()
{
    //
}

/* ************************************************************************** */

int ServoXL::getAlarmLed()
{
    // Not available on XL320
    return 0;
}

void ServoXL::setAlarmLed()
{
    // Not available on XL320
    return;
}

int ServoXL::getLock()
{
    // Not available on XL320
    return 0;
}

void ServoXL::setLock()
{
    // Not available on XL320
    return;
}

/* ************************************************************************** */

int ServoXL::getControlMode()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CONTROL_MODE)];
}

int ServoXL::getDGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_D_GAIN)];
}

int ServoXL::getIGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_I_GAIN)];
}

int ServoXL::getPGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_P_GAIN)];
}

int ServoXL::getGoalTorque()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_GOAL_TORQUE)];
}

int ServoXL::getHardwareErrorStatus()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_HW_ERROR_STATUS)];
}

/* ************************************************************************** */

void ServoXL::setId(int id)
{
    //std::cout << "[#" << servoId << "] setId(from " << servoId << " to " << id << ")" << std::endl;

    if (id > -1 && id < 253)
    {
        std::lock_guard <std::mutex> lock(access);

        // Maybe check if new ID is not already in use ?
        registerTableValues[gid(REG_ID)] = id;
        registerTableCommits[gid(REG_ID)] = 1;
    }
}

void ServoXL::setError(const int error)
{
    std::lock_guard <std::mutex> lock(access);

    // On protocol v2, the error is not a bitfield but a simple error number,
    // so we cannot keep adding errors fields

    if (error != 0)
    {
        statusError = error;
    }
}
