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
 * \file ServoEX.cpp
 * \date 23/04/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoEX.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"

ServoEX::ServoEX(int dynamixel_id, int dynamixel_model, int control_mode):
    ServoDynamixel(EX_control_table, dynamixel_id, dynamixel_model, control_mode)
{
    if (dynamixel_model == SERVO_EX106)
    {
        runningDegrees = 280;
    }
    else if (dynamixel_model == SERVO_EX106p)
    {
        runningDegrees = 251;
    }

    steps = 4096;
}

ServoEX::~ServoEX()
{
    //
}

/* ************************************************************************** */

int ServoEX::getDriveMode()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_DRIVE_MODE)];
}

int ServoEX::getCwComplianceMargin()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CW_COMPLIANCE_MARGIN)];
}

int ServoEX::getCcwComplianceMargin()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CCW_COMPLIANCE_MARGIN)];
}

int ServoEX::getCwComplianceSlope()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CW_COMPLIANCE_SLOPE)];
}

int ServoEX::getCcwComplianceSlope()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CCW_COMPLIANCE_SLOPE)];
}

int ServoEX::getSensedCurrent()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_CURRENT)];
}
