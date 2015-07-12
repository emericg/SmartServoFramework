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
 * \file ServoAX.cpp
 * \date 23/04/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoAX.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"

ServoAX::ServoAX(int dynamixel_id, int dynamixel_model, int control_mode):
    ServoDynamixel(AXDXRX_control_table, dynamixel_id, dynamixel_model, control_mode)
{
    runningDegrees = 300;
    steps = 1024;
}

ServoAX::~ServoAX()
{
    //
}

/* ************************************************************************** */

int ServoAX::getCwComplianceMargin()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CW_COMPLIANCE_MARGIN)];
}

int ServoAX::getCcwComplianceMargin()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CCW_COMPLIANCE_MARGIN)];
}

int ServoAX::getCwComplianceSlope()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CW_COMPLIANCE_SLOPE)];
}

int ServoAX::getCcwComplianceSlope()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CCW_COMPLIANCE_SLOPE)];
}
