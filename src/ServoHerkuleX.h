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
 * \file ServoHerkuleX.h
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef SERVO_HERKULEX_H
#define SERVO_HERKULEX_H

#include "Servo.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ControllerAPIs
 *  @{
 */

/*!
 * \brief The HerkuleX servo class.
 */
class ServoHerkuleX: public Servo
{
protected:
    int *registerTableValuesRAM; //!< New table used to support "dual value (ROM/RAM) registers" values
    int *registerTableCommitsRAM; //!< New table used to support "dual value (ROM/RAM) registers" commits

    int gotopos = 0;
    int gotopos_commit = 0;

public:
    ServoHerkuleX(const int control_table[][8], int herkulex_id, int herkulex_model, int speed_mode = 0);
    virtual ~ServoHerkuleX() = 0;

    // Device
    void status();
    std::string getModelString();
    void getModelInfos(int &servo_serie, int &servo_model);

    // Helpers
    void waitMovmentCompletion(int timeout_ms = 5000);

    // Getters
    int getId(const int reg_type = REGISTER_ROM);
    int getBaudRate();

    int getCwAngleLimit(const int reg_type = REGISTER_RAM); // min position
    int getCcwAngleLimit(const int reg_type = REGISTER_RAM); // max position

    double getHighestLimitTemp();
    double getHighestLimitTemp(const int reg_type = REGISTER_RAM);
    double getLowestLimitVolt();
    double getLowestLimitVolt(const int reg_type = REGISTER_RAM);
    double getHighestLimitVolt();
    double getHighestLimitVolt(const int reg_type = REGISTER_RAM);

    int getMaxTorque();
    int getStatusReturnLevel(const int reg_type = REGISTER_RAM);
    int getAlarmLed(const int reg_type = REGISTER_RAM);
    int getAlarmShutdown(const int reg_type = REGISTER_RAM);
    int getTorqueEnabled(const int reg_type = REGISTER_RAM);
    int getLed();

    int getDGain(const int reg_type = REGISTER_RAM);
    int getIGain(const int reg_type = REGISTER_RAM);
    int getPGain(const int reg_type = REGISTER_RAM);

    int getGoalPosition();
        int getGoalPositionCommited();
        void commitGoalPosition();
    int getMovingSpeed();
    int getTorqueLimit();
    int getCurrentPosition();
    int getCurrentSpeed();
    int getCurrentLoad();
    double getCurrentVoltage();
    double getCurrentTemperature();
    int getMoving();

    // Setters
    void setId(int id);
    void setCWLimit(int limit, const int reg_type = REGISTER_RAM);
    void setCCWLimit(int limit, const int reg_type = REGISTER_RAM);
    void setGoalPosition(int pos);
    void setGoalPosition(int pos, int time_budget_ms);
    // TODO //void setGoalPosition(int pos, int speed);
    // TODO //void setGoalPosition(int pos, int int max_speed, int accel_ratio);
    void moveGoalPosition(int move);
    void setMovingSpeed(int speed);
    void setMaxTorque(int torque);

    void setLed(int led);
    void setTorqueEnabled(int torque);

    // General purpose getters/setters (using generic register's name)
    int getValue(const int reg, int reg_type = REGISTER_AUTO);
    int getValueCommit(const int reg, int reg_type = REGISTER_AUTO);

    void setValue(const int reg, int value, int reg_type = REGISTER_AUTO);
    void updateValue(const int reg, int value, int reg_type = REGISTER_AUTO);
    void commitValue(const int reg, int commit, int reg_type = REGISTER_AUTO);
};

/** @}*/

#endif /* SERVO_HERKULEX_H */
