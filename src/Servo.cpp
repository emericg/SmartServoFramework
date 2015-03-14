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
 * \file Servo.cpp
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "Servo.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"
#include "ControlTablesDynamixel.h"

#include <iostream>
#include <thread>

Servo::Servo()
{
    ct = NULL;

    registerTableSize = 0;
    registerTableValues = NULL;
    registerTableCommits = NULL;

    servoId = 0;
    servoModel = 0;
    servoSerie = 0;

    steps = 0;
    runningDegrees = 0;

    commError = 0;
    statusError = 0;
    statusDetail = 0;
    valueErrors = 0;
    errorCount = 0;

    actionProgrammed = 0;
    rebootProgrammed = 0;
    refreshProgrammed = 0;
    resetProgrammed = 0;
}

Servo::~Servo()
{
    if (registerTableValues != NULL)
    {
        delete [] registerTableValues;
        registerTableValues = NULL;
    }

    if (registerTableCommits != NULL)
    {
        delete [] registerTableCommits;
        registerTableCommits = NULL;
    }
}

/* ************************************************************************** */

const int (*(Servo::getControlTable()))[8]
{
    return ct;
}

int Servo::getRegisterCount()
{
    return registerTableSize;
}

int Servo::gid(const int reg)
{
    int id = getRegisterTableIndex(ct, reg);

    // Set fallback id to 0, so nobody try to access negative table index.
    if (id < 0)
    {
        id = 0;
    }

    return id;
}

int Servo::gaddr(const int reg, const int reg_mode)
{
    return getRegisterAddr(ct, reg, reg_mode);
}

/* ************************************************************************** */

int Servo::getStatus()
{
    std::lock_guard <std::mutex> lock(access);
    return statusDetail;
}

void Servo::setStatus(const int status)
{
    std::lock_guard <std::mutex> lock(access);
    statusDetail = status;
}

int Servo::getError()
{
    std::lock_guard <std::mutex> lock(access);
    return statusError;
}

void Servo::setError(const int error)
{
    if (error > 0)
    {
        std::lock_guard <std::mutex> lock(access);
        statusError += error;
        errorCount++;
    }
}

void Servo::clearErrors()
{
    std::lock_guard <std::mutex> lock(access);

    commError = 0;
    statusError = 0;
    statusDetail = 0;
    valueErrors = 0;
    errorCount = 0;
}

int Servo::getErrorCount()
{
    std::lock_guard <std::mutex> lock(access);

    errorCount--;
    return (errorCount + 1);
}

/* ************************************************************************** */

void Servo::action()
{
    std::lock_guard <std::mutex> lock(access);
    actionProgrammed = 1;
}

void Servo::reboot()
{
    std::lock_guard <std::mutex> lock(access);
    rebootProgrammed = 1;
}

void Servo::refresh()
{
    std::lock_guard <std::mutex> lock(access);
    refreshProgrammed = 1;
}

void Servo::reset(int setting)
{
    if (setting == RESET_ALL || setting == RESET_ALL_EXCEPT_ID || setting == RESET_ALL_EXCEPT_ID_BAUDRATE)
    {
        std::lock_guard <std::mutex> lock(access);
        resetProgrammed = setting;
    }
}

void Servo::getActions(int &action, int &reboot, int &refresh, int &reset)
{
    std::lock_guard <std::mutex> lock(access);

    action = actionProgrammed;
    reboot = rebootProgrammed;
    reset = resetProgrammed;
    refresh = refreshProgrammed;

    actionProgrammed = 0;
    rebootProgrammed = 0;
    resetProgrammed = 0;
    refreshProgrammed = 0;
}

int Servo::changeInternalId(int newId)
{
    int retcode = 0;

    if (newId > 0 && newId < 254)
    {
        std::lock_guard <std::mutex> lock(access);
        servoId = newId;
        retcode = 1;
    }

    return retcode;
}

/* ************************************************************************** */

void Servo::status()
{
    std::lock_guard <std::mutex> lock(access);

    std::cout << "Basic status(#" << servoId << ")" << std::endl;

    std::cout << "> model      : " << servoModel << std::endl;
    std::cout << "> firmware   : " << registerTableValues[gid(REG_FIRMWARE_VERSION)] << std::endl;
}

int Servo::getDeviceBrand()
{
    int brand = SERVO_UNKNOWN;

    if (servoModel)
    {
        if (servoModel > SERVO_HERKULEX)
        {
            brand = SERVO_HERKULEX;
        }
        else if (servoModel > SERVO_DYNAMIXEL)
        {
            brand = SERVO_DYNAMIXEL;
        }
    }

    return brand;
}

int Servo::getDeviceSerie()
{
    return servoSerie;
}

int Servo::getDeviceModel()
{
    return servoModel;
}

/* ************************************************************************** */

int Servo::getId()
{
    // This function return the ID currently in use for the servo, and so not
    // the one in registerTableValues[gid(REG_ID)], which could be set to a new
    // value not yet commited to the device.

    std::lock_guard <std::mutex> lock(access);
    return servoId;
}

int Servo::getModelNumber()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MODEL_NUMBER)];
}

int Servo::getFirmwareVersion()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_FIRMWARE_VERSION)];
}

int Servo::getBaudNum()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_BAUD_RATE)];
}

int Servo::getCwAngleLimit()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MIN_POSITION)];
}

int Servo::getCcwAngleLimit()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MAX_POSITION)];
}

int Servo::getSteps()
{
    std::lock_guard <std::mutex> lock(access);
    return steps;
}

int Servo::getRunningDegrees()
{
    std::lock_guard <std::mutex> lock(access);
    return runningDegrees;
}

int Servo::getMaxTorque()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MAX_TORQUE)];
}

int Servo::getStatusReturnLevel()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_STATUS_RETURN_LEVEL)];
}

int Servo::getAlarmLed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_ALARM_LED)];
}

int Servo::getAlarmShutdown()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_ALARM_SHUTDOWN)];
}

int Servo::getTorqueEnabled()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_TORQUE_ENABLE)];
}

int Servo::getLed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_LED)];
}

int Servo::getCurrentPosition()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_POSITION)];
}

int Servo::getCurrentSpeed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_SPEED)];
}

int Servo::getCurrentLoad()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_LOAD)];
}

/* ************************************************************************** */

void Servo::setId(int id)
{
    //std::cout << "[#" << servoId << "] setId(from " << servoId << " to " << id << ")" << std::endl;

    if (id > -1 && id < 254)
    {
        std::lock_guard <std::mutex> lock(access);

        // Maybe check if new ID is not already in use ?
        registerTableValues[gid(REG_ID)] = id;
        registerTableCommits[gid(REG_ID)] = 1;
    }
}

void Servo::setCWLimit(int limit)
{
    //std::cout << "[#" << servoId << "] setCWLimit(" << limit << ")" << std::endl;

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MIN_POSITION)] = limit;
        registerTableCommits[gid(REG_MIN_POSITION)] = 1;
    }
}

void Servo::setCCWLimit(int limit)
{
    //std::cout << "[#" << servoId << "] setCCWLimit(" << limit << ")" << std::endl;

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_POSITION)] = limit;
        registerTableCommits[gid(REG_MAX_POSITION)] = 1;
    }
}

/* ************************************************************************** */

int Servo::getValue(int reg_name, int reg_type)
{
    int value = -1;

    // Find register's informations (addr, size...)
    RegisterInfos infos;
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            std::lock_guard <std::mutex> lock(access);

            // Get value
            value = registerTableValues[infos.reg_index];
        }
        else
        {
            std::cerr << "[#" << servoId << "] getValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ID ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << servoId << "] getValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
    }

    return value;
}

int Servo::getValueCommit(int reg_name, int reg_type)
{
    int commit = -1;

    // Find register's informations (addr, size...)
    RegisterInfos infos;
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            std::lock_guard <std::mutex> lock(access);

            // Get value
            commit = registerTableCommits[infos.reg_index];
        }
        else
        {
            std::cerr << "[#" << servoId << "] getValueCommit(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ID ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << servoId << "] getValueCommit(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
    }

    return commit;
}

/* ************************************************************************** */

void Servo::setValue(int reg_name, int reg_value, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos;
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            // Check if we have permission to write into this register
            if (infos.reg_access_mode == READ_WRITE)
            {
                // Check value
                if (reg_value >= infos.reg_value_min && reg_value <= infos.reg_value_max)
                {
                    std::lock_guard <std::mutex> lock(access);

                    // Set value
                    registerTableValues[infos.reg_index] = reg_value;
                    registerTableCommits[infos.reg_index] = 1;
                }
                else
                {
                    std::cerr << "[#" << servoId << "] setValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << " to '" << reg_value
                              << "')  [VALUE ERROR]! min/max(" << infos.reg_value_min << "/" << infos.reg_value_max << ")" << std::endl;
                }
            }
            else
            {
                std::cerr << "[#" << servoId << "] setValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ACCESS ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << servoId << "] setValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ID ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << servoId << "] setValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
    }
}

void Servo::updateValue(int reg_name, int reg_value, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos;
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            // Check value
            if (reg_value >= infos.reg_value_min && reg_value <= infos.reg_value_max)
            {
                std::lock_guard <std::mutex> lock(access);

                // Update value
                registerTableValues[infos.reg_index] = reg_value;
            }
            else
            {
                valueErrors++;
                errorCount++;

                std::cerr << "[#" << servoId << "] updateValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << " to '" << reg_value
                          << "')  [VALUE ERROR]! min/max(" << infos.reg_value_min << "/" << infos.reg_value_max << ")" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << servoId << "] updateValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ID ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << servoId << "] updateValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
    }
}

void Servo::commitValue(int reg_name, int commit, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos;
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            // Check if we have permission to write into this register
            if (infos.reg_access_mode == READ_WRITE)
            {
                if (commit != 0)
                {
                    commit = 1;
                }

                std::lock_guard <std::mutex> lock(access);

                // Set value
                registerTableCommits[infos.reg_index] = commit;
            }
            else
            {
                std::cerr << "[#" << servoId << "] commitValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ACCESS ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << servoId << "] commitValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ID ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << servoId << "] commitValue(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
    }
}

/* ************************************************************************** */
