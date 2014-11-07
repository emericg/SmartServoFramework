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
 * \file ServoXL.h
 * \date 08/07/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERVO_XL_H
#define SERVO_XL_H

#include "ServoDynamixel.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ControllerAPIs
 *  @{
 */

/*!
 * \brief XL servo serie.
 *
 * This servo serie use the new "protocol v2", also used by the Dynamixel PRO serie.
 * More informations about them on Robotis website:
 * - http://support.robotis.com/en/product/dynamixel/xl-320.htm
 */
class ServoXL: public ServoDynamixel
{
public:
    ServoXL(int dynamixel_id, int dynamixel_model, int control_mode = SPEED_MANUAL);
    ~ServoXL();

    // Not available on XL-320:
    int getAlarmLed();
    void setAlarmLed();
    int getLock();
    void setLock();

    // Getters
    int getControlMode(); // Only on XL-320
    int getDGain();
    int getIGain();
    int getPGain();
    int getGoalTorque();
    int getHardwareErrorStatus(); // Only on XL-320

    // Setters
    void setId(int id); //! XL-320 ids are in range [0;252] instead of [0;253]
    void setError(const int error);
};

/** @}*/

#endif /* SERVO_XL_H */
