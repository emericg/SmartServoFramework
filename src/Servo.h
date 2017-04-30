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
 * \file Servo.h
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef SERVO_H
#define SERVO_H

#include "ControlTables.h"

#include <string>
#include <map>
#include <mutex>

/** \addtogroup ManagedAPIs
 *  @{
 */

/*!
 * \brief The SpeedMode enum
 *
 * Used to indicate if we want "automatic" or manual speed control. Automatic
 * speed control regulate the speed relative to the movment amplitude.
 */
enum SpeedMode_e {
    SPEED_MANUAL = 0,
    SPEED_AUTO   = 1
};

/*!
 * \brief The "Servo" device base class.
 */
class Servo
{
protected:
    std::mutex access;              //!< Lock servo to avoid concurrent use by controller and user

    const int (*ct)[8] = nullptr;   //!< Pointer to the control table for a given servo class (selected by the constructor)

    int registerTableSize = 0;      //!< Number of register in the servo control table
    int *registerTableValues = nullptr;
    int *registerTableCommits = nullptr;

    int servoId = 0;
    int servoModel = 0;
    int servoSerie = 0;

    int steps = 0;                  //!< Number of step the servo can handle (depends on the serie)
    int runningDegrees = 0;         //!< The amplitude of movement (a device with less than 360 'running degree' has a dead zone)

    int commError = 0;              //!< Error code from the serial link (when communicating with this particular device)
    int statusError = 0;            //!< Error bitfield from the device
    int statusDetail = 0;           //!< Additional status bitfield from the device (only available on HerkuleX devices)
    int valueErrors = 0;            //!< Register value boundaries error count
    int errorCount = 0;             //!< Global error count

    int actionProgrammed = 0;
    int rebootProgrammed = 0;
    int refreshProgrammed = 0;
    int resetProgrammed = 0;

public:
    Servo();
    virtual ~Servo() = 0;

    // Settings
    const int (*getControlTable())[8];
    int getRegisterCount();
    int gid(const int reg);
    int gaddr(const int reg, const int reg_mode = REGISTER_AUTO);

    // Device
    virtual void status();
    virtual std::string getModelString() = 0;
    virtual void getModelInfos(int &servo_serie, int &servo_model) = 0;
    int getDeviceBrand();
    int getDeviceSerie();
    int getDeviceModel();

    // Error handling
    int getStatus();
    virtual void setStatus(const int status);
    int getError();
    virtual void setError(const int error);
    void clearErrors();
    int getErrorCount();

    // Actions
    void action();
    void reboot();
    void reset(int setting);
    void refresh();
    void getActions(int &action, int &reboot, int &refresh, int &reset);

    // Helpers
    int changeInternalId(int newId);
    virtual void setGoalPosition(int pos, int time_budget_ms) = 0;
    virtual void waitMovementCompletion(int timeout_ms = 5000) = 0;

    // Getters
    virtual int getId();
    virtual int getModelNumber();
    virtual int getFirmwareVersion();
    virtual int getBaudNum();
    virtual int getBaudRate() = 0;

    virtual int getCwAngleLimit(); // min position
    virtual int getCcwAngleLimit(); // max position
    int getSteps();
    int getRunningDegrees();

    virtual double getHighestLimitTemp() = 0;
    virtual double getLowestLimitVolt() = 0;
    virtual double getHighestLimitVolt() = 0;
    int getMaxTorque();
    int getStatusReturnLevel();
    int getAlarmLed();
    int getAlarmShutdown();
    int getTorqueEnabled();
    int getLed();

    virtual int getGoalPosition() = 0;
    virtual int getMovingSpeed() = 0;

    int getCurrentPosition();
    int getCurrentSpeed();
    int getCurrentLoad();
    virtual double getCurrentVoltage() = 0;
    virtual double getCurrentTemperature() = 0;
    virtual int getMoving() = 0;

    // Setters
    virtual void setId(int id);
    virtual void setCWLimit(int limit);
    virtual void setCCWLimit(int limit);
    virtual void setGoalPosition(int pos) = 0;

    virtual void setLed(int led) = 0;
    virtual void setTorqueEnabled(int torque) = 0;

    // General purpose getters/setters (using generic register's name)
    virtual int getValue(const int reg_reg, int reg_type = REGISTER_AUTO);
    virtual int getValueCommit(const int reg_reg, int reg_type = REGISTER_AUTO);

    virtual void setValue(const int reg_reg, int reg_value, int reg_type = REGISTER_AUTO);
    virtual void updateValue(const int reg_reg, int reg_value, int reg_type = REGISTER_AUTO);
    virtual void commitValue(const int reg_reg, int commit, int reg_type = REGISTER_AUTO);
};

/** @}*/

#endif // SERVO_H
