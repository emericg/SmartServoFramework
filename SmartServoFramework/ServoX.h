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
 * \file ServoX.h
 * \date 29/04/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef SERVO_X_H
#define SERVO_X_H

#include "ServoDynamixel.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ManagedAPIs
 *  @{
 */

/*!
 * \brief XM/XH servo series.
 * \ref ServoDynamixel
 * \ref XMXH_control_table
 * \note The X series use the new "protocol v2", also used by the Dynamixel PRO serie.

 * More informations about them on Robotis website:
 * - http://www.robotis.us/x-series/
 * - http://en.robotis.com/index/product.php?cate_code=101210
 * - http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series.htm
 */
class ServoX: public ServoDynamixel
{
public:
    ServoX(int dynamixel_id, int dynamixel_model, int control_mode = SPEED_MANUAL);
    ~ServoX();

    // Not available on X series:
    int getAlarmLed();
    void setAlarmLed();
    int getLock();
    void setLock();

    // Getters
    int getControlMode(); // Only on X series
    int getDGain();
    int getIGain();
    int getPGain();
    int getHardwareErrorStatus(); // Only on X series

    // Setters
    void setId(int id); //! X series ids are in range [0;252] instead of [0;253]
    void setError(const int error);
};

/** @}*/

#endif // SERVO_X_H
