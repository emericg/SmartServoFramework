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
 * \file ServoDynamixel.h
 * \date 13/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERVO_DYNAMIXEL_H
#define SERVO_DYNAMIXEL_H

#include "Servo.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ControllerAPIs
 *  @{
 */

/*!
 * \brief The Dynamixel servo class.
 */
class ServoDynamixel: public Servo
{
    int speedMode;  //!< Control the servo with manual or 'automatic' speed mode, using '::SpeedMode_e' enum.

public:
    ServoDynamixel(const int control_table[][8], int dynamixel_id, int dynamixel_model, int speed_mode = SPEED_MANUAL);
    virtual ~ServoDynamixel() = 0;

    // Device
    void status();
    std::string getModelString();
    void getModelInfos(int &servo_serie, int &servo_model);

    // Helpers
    int getSpeedMode();
    void setSpeedMode(int speed_mode);
    void waitMovmentCompletion(int timeout_ms = 5000);

    // Getters
    int getBaudRate();
    int getReturnDelay();

    double getHighestLimitTemp();
    double getLowestLimitVolt();
    double getHighestLimitVolt();

    int getMaxTorque();

    int getGoalPosition();
    int getMovingSpeed();

    int getTorqueLimit();
    int getCurrentPosition();
    int getCurrentSpeed();
    int getCurrentLoad();
    double getCurrentVoltage();
    double getCurrentTemperature();

    int getRegistered();
    int getMoving();
    int getLock();
    int getPunch();

    // Setters
    void setId(int id);
    void setCWLimit(int limit);
    void setCCWLimit(int limit);
    void setGoalPosition(int pos);
    void setGoalPosition(int pos, int time_budget_ms);
    // TODO //void setGoalPosition(int pos, int speed);
    // TODO //void setGoalPosition(int pos, int int max_speed, int accel_ratio);
    void moveGoalPosition(int move);
    void setMovingSpeed(int speed);
    void setMaxTorque(int torque);

    void setLed(int led);
    void setTorqueEnabled(int torque);
};

/** @}*/

#endif /* SERVO_DYNAMIXEL_H */
