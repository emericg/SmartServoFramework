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
 * \file ServoMX.cpp
 * \date 23/04/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoMX.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"

ServoMX::ServoMX(int dynamixel_id, int dynamixel_model, int control_mode):
    ServoDynamixel(MX_control_table, dynamixel_id, dynamixel_model, control_mode)
{
    runningDegrees = 360;
    steps = 4096;
}

ServoMX::~ServoMX()
{
    //
}

/* ************************************************************************** */

int ServoMX::getDriveMode()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_DRIVE_MODE)];
}

int ServoMX::getMultiTurnOffset()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MULTI_TURN_OFFSET)];
}

int ServoMX::getResolutionDivider()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_RESOLUTION_DIVIDER)];
}

int ServoMX::getDGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_D_GAIN)];
}

int ServoMX::getIGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_I_GAIN)];
}

int ServoMX::getPGain()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_P_GAIN)];
}

int ServoMX::getConsumingCurrent()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_CURRENT)];
}

int ServoMX::getTorqueControlMode()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CONTROL_MODE)];
}

int ServoMX::getGoalTorque()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_GOAL_TORQUE)];
}

int ServoMX::getGoalAccel()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_GOAL_ACCELERATION)];
}
