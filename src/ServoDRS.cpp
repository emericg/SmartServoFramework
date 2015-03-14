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
 * \file ServoDRS.cpp
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoDRS.h"

#include "HerkuleX.h"
#include "HerkuleXTools.h"
#include "ControlTables.h"
#include "ControlTablesHerkuleX.h"

#include <iostream>
#include <string.h>

ServoDRS::ServoDRS(int herkulex_id, int herkulex_model, int control_mode):
    ServoHerkuleX(getRegisterTable(hkx_get_servo_model(herkulex_model)), herkulex_id, herkulex_model, control_mode)
{
    int servo_model = hkx_get_servo_model(herkulex_model);

    if (servo_model == SERVO_DRS_0402 || servo_model == SERVO_DRS_0602)
    {
        runningDegrees = 360;
        steps = 12962;
    }
    else if (servo_model == SERVO_DRS_0401 || servo_model == SERVO_DRS_0601)
    {
        runningDegrees = 320;
        steps = 2048;
    }
    else //if (servo_model == SERVO_DRS_0101 || servo_model == SERVO_DRS_0201)
    {
        runningDegrees = 320;
        steps = 1024;
    }
}

ServoDRS::~ServoDRS()
{
    //
}
