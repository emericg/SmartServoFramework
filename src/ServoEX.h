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
 * \file ServoEX.h
 * \date 23/04/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERVO_EX_H
#define SERVO_EX_H

#include "ServoDynamixel.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ControllerAPIs
 *  @{
 */

/*!
 * \brief EX servo serie.
 *
 * More informations about them on Robotis website:
 * - http://support.robotis.com/en/product/dynamixel/dxl_ex_main.htm
 */
class ServoEX: public ServoDynamixel
{
public:
    ServoEX(int dynamixel_id, int dynamixel_model, int control_mode = SPEED_MANUAL);
    ~ServoEX();

    // Getters
    int getDriveMode();

    int getCwComplianceMargin();
    int getCcwComplianceMargin();
    int getCwComplianceSlope();
    int getCcwComplianceSlope();

    int getSensedCurrent();
};

/** @}*/

#endif /* SERVO_EX_H */
