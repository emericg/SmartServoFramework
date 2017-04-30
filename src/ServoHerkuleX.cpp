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
 * \file ServoHerkuleX.cpp
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoHerkuleX.h"
#include "minitraces.h"

#include "HerkuleX.h"
#include "HerkuleXTools.h"
#include "ControlTablesHerkuleX.h"

#include <cstring>

ServoHerkuleX::ServoHerkuleX(const int control_table[][8], int herkulex_id, int herkulex_model, int speed_mode):
    Servo()
{
    // Store servo id, serie and model
    servoId = herkulex_id;
    hkx_get_model_infos(herkulex_model, servoSerie, servoModel);

    // Set the control table
    if (control_table)
    {
        ct = control_table;
    }
    else
    {
        ct = DRS0101_control_table;
    }

    // Register count // FIXME // Horrible hack!
    for (registerTableSize = 0; registerTableSize < 64; registerTableSize++)
    {
        if (ct[registerTableSize][0] == 999) // end of table
        {
            break;
        }
    }

    // Init register tables (value and commit info) with value-initialization
    registerTableValues = new int [registerTableSize]();
    registerTableCommits = new int [registerTableSize]();

    // New table to support "dual value ROM/RAM registers"
    registerTableValuesRAM = new int [registerTableSize]();
    registerTableCommitsRAM = new int [registerTableSize]();

    gotopos = 0;
    gotopos_commit = 0;

    // Set model and id because we already known them
    registerTableValues[gid(REG_MODEL_NUMBER)] = herkulex_model;
    if (herkulex_id > -1 && herkulex_id < BROADCAST_ID)
    {
        registerTableValues[gid(REG_ID)] = herkulex_id;
    }

    // Set default value for ack policy, ACK_REPLY_READ for HerkuleX devices
    // (will be overwritten when reading the real value from the device)
    registerTableValues[gid(REG_STATUS_RETURN_LEVEL)] = ACK_REPLY_READ;
    registerTableValuesRAM[gid(REG_STATUS_RETURN_LEVEL)] = ACK_REPLY_READ;
}

ServoHerkuleX::~ServoHerkuleX()
{
    if (registerTableValues != nullptr)
    {
        delete [] registerTableValues;
        registerTableValues = nullptr;
    }

    if (registerTableCommits != nullptr)
    {
        delete [] registerTableCommits;
        registerTableCommits = nullptr;
    }

    if (registerTableValuesRAM != nullptr)
    {
        delete [] registerTableValuesRAM;
        registerTableValuesRAM = nullptr;
    }

    if (registerTableCommitsRAM != nullptr)
    {
        delete [] registerTableCommitsRAM;
        registerTableCommitsRAM = nullptr;
    }
}

/* ************************************************************************** */

void ServoHerkuleX::status()
{
    std::lock_guard <std::mutex> lock(access);

    TRACE_INFO(HKX, "Status(#%i)", servoId);

    TRACE_INFO(HKX, "> model      : %i", servoModel);
    TRACE_INFO(HKX, "> firmware   : %i", registerTableValues[gid(REG_FIRMWARE_VERSION)]);
    TRACE_INFO(HKX, "> baudrate   : %i", hkx_get_baudrate(registerTableValues[gid(REG_BAUD_RATE)]));

    TRACE_INFO(HKX, ">> steps          : %i", steps);
    TRACE_INFO(HKX, ">> runningDegrees : %i", runningDegrees);

    TRACE_INFO(HKX, "> torque enabled  : %i", registerTableValues[gid(REG_TORQUE_ENABLE)]);
    TRACE_INFO(HKX, "> max torque      : %i", registerTableValues[gid(REG_MAX_TORQUE)]);
    TRACE_INFO(HKX, "> torque limit    : %i", registerTableValues[gid(REG_TORQUE_LIMIT)]);

    TRACE_INFO(HKX, "> goal position   : %i", registerTableValues[gid(REG_GOAL_POSITION)]);
    TRACE_INFO(HKX, "> goal speed      : %i", registerTableValues[gid(REG_GOAL_SPEED)]);
    TRACE_INFO(HKX, "> current position: %i", registerTableValues[gid(REG_CURRENT_POSITION)]);
    TRACE_INFO(HKX, "> current speed   : %i", registerTableValues[gid(REG_CURRENT_SPEED)]);
    TRACE_INFO(HKX, "> current load    : %i", registerTableValues[gid(REG_CURRENT_LOAD)]);
    TRACE_INFO(HKX, "> current voltage : %i", registerTableValues[gid(REG_CURRENT_VOLTAGE)]);
    TRACE_INFO(HKX, "> current temp    : %i", registerTableValues[gid(REG_CURRENT_TEMPERATURE)]);
    TRACE_INFO(HKX, "> registered      : %i", registerTableValues[gid(REG_REGISTERED)]);
    TRACE_INFO(HKX, "> moving          : %i", registerTableValues[gid(REG_MOVING)]);
    TRACE_INFO(HKX, "> lock            : %i", registerTableValues[gid(REG_LOCK)]);
    TRACE_INFO(HKX, "> punch           : %i", registerTableValues[gid(REG_PUNCH)]);
}

std::string ServoHerkuleX::getModelString()
{
    std::lock_guard <std::mutex> lock(access);
    return hkx_get_model_name(registerTableValues[gid(REG_MODEL_NUMBER)]);
}

void ServoHerkuleX::getModelInfos(int &servo_serie, int &servo_model)
{
    std::lock_guard <std::mutex> lock(access);
    int model_number = registerTableValues[gid(REG_MODEL_NUMBER)];

    hkx_get_model_infos(model_number, servo_serie, servo_model);
}

/* ************************************************************************** */

void ServoHerkuleX::waitMovementCompletion(int timeout_ms)
{
    // TODO
}

/* ************************************************************************** */

int ServoHerkuleX::getId(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    // If asked for (reg_type == REGISTER_RAM), this function return the ID currently
    // in use for the servo, and so not the one in registerTableValuesRAM[gid(REG_ID)],
    // which could be set to a new value not yet commited to the device.
    int id = servoId;

    if (reg_type == REGISTER_ROM)
    {
        id = registerTableValues[gid(REG_ID)];
    }

    return id;
}

int ServoHerkuleX::getBaudRate()
{
    std::lock_guard <std::mutex> lock(access);
    return hkx_get_baudrate(registerTableValues[gid(REG_BAUD_RATE)], servoSerie);
}

int ServoHerkuleX::getCwAngleLimit(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_MIN_POSITION)];
    else
        return registerTableValues[gid(REG_MIN_POSITION)];
}

int ServoHerkuleX::getCcwAngleLimit(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_MAX_POSITION)];
    else
        return registerTableValues[gid(REG_MAX_POSITION)];
}

double ServoHerkuleX::getHighestLimitTemp()
{
    return getHighestLimitTemp(REGISTER_ROM);
}

double ServoHerkuleX::getHighestLimitTemp(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);
    int temp = 0;

    if (reg_type == REGISTER_RAM)
        temp = registerTableValuesRAM[gid(REG_TEMPERATURE_LIMIT)];
    else
        temp = registerTableValues[gid(REG_TEMPERATURE_LIMIT)];

    // FIXME temperature is using a non linear scale
    return (temp * 0.326);
}

double ServoHerkuleX::getLowestLimitVolt()
{
    return getLowestLimitVolt(REGISTER_ROM);
}

double ServoHerkuleX::getLowestLimitVolt(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);
    int volt = 0;

    if (reg_type == REGISTER_RAM)
        volt = registerTableValuesRAM[gid(REG_VOLTAGE_LOWEST_LIMIT)];
    else
        volt = registerTableValues[gid(REG_VOLTAGE_LOWEST_LIMIT)];

    return (volt * 0.074074);
}

double ServoHerkuleX::getHighestLimitVolt()
{
    return getHighestLimitVolt(REGISTER_ROM);
}

double ServoHerkuleX::getHighestLimitVolt(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);
    int volt = 0;

    if (reg_type == REGISTER_RAM)
        volt = registerTableValuesRAM[gid(REG_VOLTAGE_HIGHEST_LIMIT)];
    else
        volt = registerTableValues[gid(REG_VOLTAGE_HIGHEST_LIMIT)];

    return (volt * 0.074074);
}
/*
int ServoHerkuleX::getMaxTorque()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(SERVO_MAX_TORQUE)];
}
*/
int ServoHerkuleX::getStatusReturnLevel(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_STATUS_RETURN_LEVEL)];
    else
        return registerTableValues[gid(REG_STATUS_RETURN_LEVEL)];
}

int ServoHerkuleX::getAlarmLed(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_ALARM_LED)];
    else
        return registerTableValues[gid(REG_ALARM_LED)];
}

int ServoHerkuleX::getAlarmShutdown(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_ALARM_SHUTDOWN)];
    else
        return registerTableValues[gid(REG_ALARM_SHUTDOWN)];
}

int ServoHerkuleX::getLed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValuesRAM[gid(REG_LED)];
}

int ServoHerkuleX::getTorqueEnabled(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_TORQUE_ENABLE)];
    else
        return registerTableValues[gid(REG_TORQUE_ENABLE)];
}

int ServoHerkuleX::getDGain(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_D_GAIN)];
    else
        return registerTableValues[gid(REG_D_GAIN)];
}

int ServoHerkuleX::getIGain(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_I_GAIN)];
    else
        return registerTableValues[gid(REG_I_GAIN)];
}

int ServoHerkuleX::getPGain(const int reg_type)
{
    std::lock_guard <std::mutex> lock(access);

    if (reg_type == REGISTER_RAM)
        return registerTableValuesRAM[gid(REG_P_GAIN)];
    else
        return registerTableValues[gid(REG_P_GAIN)];
}

int ServoHerkuleX::getGoalPosition()
{
    std::lock_guard <std::mutex> lock(access);
    return gotopos;
}

int ServoHerkuleX::getGoalPositionCommited()
{
    std::lock_guard <std::mutex> lock(access);
    return gotopos_commit;
}

void ServoHerkuleX::commitGoalPosition()
{
    std::lock_guard <std::mutex> lock(access);
    gotopos_commit = 0;
}

int ServoHerkuleX::getMovingSpeed()
{
    std::lock_guard <std::mutex> lock(access);
    return 0; // FIXME // registerTableValues[gid(SERVO_GOAL_SPEED)];
}
/*
int ServoHerkuleX::getTorqueLimit()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(SERVO_TORQUE_LIMIT)];
}
*/
int ServoHerkuleX::getCurrentPosition()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValuesRAM[gid(REG_ABSOLUTE_POSITION)];
}
/*
int ServoHerkuleX::getCurrentSpeed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(SERVO_CURRENT_SPEED)];
}

int ServoHerkuleX::getCurrentLoad()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(SERVO_CURRENT_LOAD)];
}
*/
double ServoHerkuleX::getCurrentVoltage()
{
    std::lock_guard <std::mutex> lock(access);
    int volt = registerTableValuesRAM[gid(REG_CURRENT_VOLTAGE)];

    return (volt * 0.074074);
}

double ServoHerkuleX::getCurrentTemperature()
{
    std::lock_guard <std::mutex> lock(access);
    int temp = registerTableValuesRAM[gid(REG_CURRENT_TEMPERATURE)];

    // FIXME temperature is using a non linear scale
    return (temp * 0.326);
}

int ServoHerkuleX::getMoving()
{
    //std::lock_guard <std::mutex> lock(access);
    return 0;//registerTableValues[gid(SERVO_MOVING)];
}

/* ************************************************************************** */

void ServoHerkuleX::setId(int id)
{
    TRACE_1(HKX, "[#%i] setId(from %i to %i)", servoId, servoId, id);

    // TODO use maxId
    if (id > -1 && id < 254)
    {
        std::lock_guard <std::mutex> lock(access);

        // Maybe check if new ID is not already in use ?
        registerTableValues[gid(REG_ID)] = id;
        registerTableCommits[gid(REG_ID)] = 1;
    }
}

void ServoHerkuleX::setCWLimit(int limit, const int reg_type)
{
    TRACE_1(HKX, "[#%i] setCWLimit(%i)", servoId, limit);

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MIN_POSITION)] = limit;
        registerTableCommits[gid(REG_MIN_POSITION)] = 1;
    }
}

void ServoHerkuleX::setCCWLimit(int limit, const int reg_type)
{
    TRACE_1(HKX, "[#%i] setCCWLimit(%i)", servoId, limit);

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_POSITION)] = limit;
        registerTableCommits[gid(REG_MAX_POSITION)] = 1;
    }
}

void ServoHerkuleX::setGoalPosition(int pos)
{
    TRACE_1(HKX, "[#%i] setGoalPosition(%i)", servoId, pos);

    if (pos > -1 && pos < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        gotopos = pos;
        gotopos_commit = 1;
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] setGoalPosition(%i > %i) [VALUE ERROR]", servoId, registerTableValues[gid(REG_CURRENT_POSITION)], pos);
    }
}

void ServoHerkuleX::setGoalPosition(int pos, int time_budget_ms)
{
    // TODO
}

void ServoHerkuleX::setLed(int led)
{
    TRACE_1(HKX, "[#%i] setLed(%i)", servoId, led);
    int color = 0;

    // Normalize value
    if (led >= 1)
    {
        if (led & LED_GREEN)
        { color += 0x01; }
        if (led & LED_BLUE)
        { color += 0x02; }
        if (led & LED_RED)
        { color += 0x04; }
    }
    else
    {
        color = 0;
    }

    std::lock_guard <std::mutex> lock(access);

    registerTableValuesRAM[gid(REG_LED)] = color;
    registerTableCommitsRAM[gid(REG_LED)] = 1;
}

void ServoHerkuleX::setTorqueEnabled(int torque)
{
    TRACE_1(HKX, "[#%i] setTorqueEnabled(%i)", servoId, torque);

    // Normalize value, 1 means 'torque on' for Dynamixel devices
    if (torque == 1)
    {
        torque = 0x60;
    }

    // Valid torque are in range [0:254]
    if (torque == 0x00 || torque == 0x40 || torque == 0x60)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValuesRAM[gid(REG_TORQUE_ENABLE)] = torque;
        registerTableCommitsRAM[gid(REG_TORQUE_ENABLE)] = 1;
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i]  setTorqueEnabled(%i) [VALUE ERROR]", servoId, torque);

    }
}

void ServoHerkuleX::moveGoalPosition(int move)
{
/*
    TRACE_1(HKX, "[#%i] moveGoalPosition(%i)", servoId, move);

    std::lock_guard <std::mutex> lock(access);
    int curr = registerTableValues[gid(SERVO_CURRENT_POSITION)];
    int newpos = registerTableValues[gid(SERVO_CURRENT_POSITION)] + move;

    // Wheel mode ?
    if (registerTableValues[gid(SERVO_MIN_POSITION)] == 0 && registerTableValues[gid(SERVO_MAX_POSITION)] == 0)
    {
        if (newpos < 0 || newpos > steps)
        {
            int mod = newpos % steps;

            TRACE_ERROR(HKX, "[#%i]  moveGoalPosition([%i > %i]) [VALUE ERROR] with modulo: %i", servoId, curr, newpos, mod);
        }
    }

    if (newpos > 0 && newpos < steps)
    {
        registerTableValues[gid(SERVO_GOAL_POSITION)] = registerTableValues[gid(SERVO_CURRENT_POSITION)] + move;
        registerTableCommits[gid(SERVO_GOAL_POSITION)] = 1;
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i]  moveGoalPosition([%i > %i]) [VALUE ERROR]", servoId, curr, newpos);
    }
*/
}
/*
void ServoHerkuleX::setMovingSpeed(int speed)
{
    TRACE_1(HKX, "[#%i] setMovingSpeed(%i)", servoId, speed);

    std::lock_guard <std::mutex> lock(access);

    if (registerTableValues[gid(SERVO_MIN_POSITION)] == 0 &&
        registerTableValues[gid(SERVO_MAX_POSITION)] == 0)
    {
        if (speed < 2048)
        {
            registerTableValues[gid(SERVO_GOAL_SPEED)] = speed;
            registerTableCommits[gid(SERVO_GOAL_SPEED)] = 1;
        }
    }
    else
    {
        if (speed < 1024)
        {
            registerTableValues[gid(SERVO_GOAL_SPEED)] = speed;
            registerTableCommits[gid(SERVO_GOAL_SPEED)] = 1;
        }
    }
}

void ServoHerkuleX::setMaxTorque(int torque)
{
    TRACE_1(HKX, "[#%i] setMovingSpeed(%i)", servoId, speed);

    if (torque < 1024)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(SERVO_MAX_TORQUE)] = torque;
        registerTableCommits[gid(SERVO_MAX_TORQUE)] = 1;
    }
}
*/
/* ************************************************************************** */

int ServoHerkuleX::getValue(int reg_name, int reg_type)
{
    int value = -1;

    // Find register's informations (addr, size...)
    RegisterInfos infos = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            if (reg_type == REGISTER_AUTO)
            {
                if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                    reg_type = REGISTER_RAM; // RAM is set to default
                else if (infos.reg_addr_rom >= 0)
                    reg_type = REGISTER_ROM;
                else if (infos.reg_addr_ram >= 0)
                    reg_type = REGISTER_RAM;
            }

            std::lock_guard <std::mutex> lock(access);

            // Get value
            if (reg_type == REGISTER_RAM)
                value = registerTableValuesRAM[infos.reg_index];
            else
                value = registerTableValues[infos.reg_index];
        }
        else
        {
            TRACE_ERROR(HKX, "[#%i] getValue(reg %i / %s) [REGISTER ID ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
        }
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] getValue(reg %i / %s) [REGISTER NAME ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
    }

    return value;
}

int ServoHerkuleX::getValueCommit(int reg_name, int reg_type)
{
    int commit = -1;

    // Find register's informations (addr, size...)
    RegisterInfos infos = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            if (reg_type == REGISTER_AUTO)
            {
                if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                    reg_type = REGISTER_BOTH;
                else if (infos.reg_addr_rom >= 0)
                    reg_type = REGISTER_ROM;
                else if (infos.reg_addr_ram >= 0)
                    reg_type = REGISTER_RAM;
            }

            std::lock_guard <std::mutex> lock(access);

            // Get value
            if (reg_type == REGISTER_ROM)
            {
                commit = registerTableCommits[infos.reg_index];
            }
            else // RAM is set to default fallback
            {
                commit = registerTableCommitsRAM[infos.reg_index];
            }
        }
        else
        {
            TRACE_ERROR(HKX, "[#%i] getValueCommit(reg %i / %s) [REGISTER ID ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
        }
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] getValueCommit(reg %i / %s) [REGISTER NAME ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
    }

    return commit;
}

/* ************************************************************************** */

void ServoHerkuleX::setValue(int reg_name, int reg_value, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
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

                    if (reg_type == REGISTER_AUTO)
                    {
                        if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                            reg_type = REGISTER_BOTH;
                        else if (infos.reg_addr_rom >= 0)
                            reg_type = REGISTER_ROM;
                        else if (infos.reg_addr_ram >= 0)
                            reg_type = REGISTER_RAM;
                    }

                    // Set value(s)
                    if (reg_type == REGISTER_ROM || reg_type == REGISTER_BOTH)
                    {
                        registerTableValues[infos.reg_index] = reg_value;
                        registerTableCommits[infos.reg_index] = 1;
                    }

                    if (reg_type == REGISTER_RAM || reg_type == REGISTER_BOTH)
                    {
                        registerTableValuesRAM[infos.reg_index] = reg_value;
                        registerTableCommitsRAM[infos.reg_index] = 1;
                    }
                }
                else
                {
                    TRACE_ERROR(HKX, "[#%i] setValue(reg %i / %s to %i) [REGISTER VALUE ERROR] (min: %i / max: %i)", servoId,
                                reg_name, getRegisterNameTxt(reg_name).c_str(), reg_value, infos.reg_value_min, infos.reg_value_max);
                }
            }
            else
            {
                TRACE_ERROR(HKX, "[#%i] setValue(reg %i / %s) [REGISTER ACCESS ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
            }
        }
        else
        {
            TRACE_ERROR(HKX, "[#%i] setValue(reg %i / %s) [REGISTER ID ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
        }
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] setValue(reg %i / %s) [REGISTER NAME ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
    }
}

void ServoHerkuleX::updateValue(int reg_name, int reg_value, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    if (getRegisterInfos(ct, reg_name, infos) == 1)
    {
        if (infos.reg_index >= 0)
        {
            // Check value
            if (reg_value >= infos.reg_value_min && reg_value <= infos.reg_value_max)
            {
                std::lock_guard <std::mutex> lock(access);

                if (reg_type == REGISTER_AUTO)
                {
                    if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_BOTH;
                    else if (infos.reg_addr_rom >= 0)
                        reg_type = REGISTER_ROM;
                    else if (infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_RAM;
                }

                // Update value(s)
                if (reg_type == REGISTER_ROM || reg_type == REGISTER_BOTH)
                {
                    registerTableValues[infos.reg_index] = reg_value;
                }

                if (reg_type == REGISTER_RAM || reg_type == REGISTER_BOTH)
                {
                    registerTableValuesRAM[infos.reg_index] = reg_value;
                }
            }
            else
            {
                valueErrors++;
                errorCount++;

                // If the value is out of bound but correspond to an error code [-1;-7], no need to print it as a 'REGISTER VALUE ERROR'
                if ( !(reg_value < 0 && reg_value > -8) )
                {
                    TRACE_ERROR(HKX, "[#%i] updateValue(reg %i / %s to %i) [REGISTER VALUE ERROR] (min: %i / max: %i)", servoId,
                                reg_name, getRegisterNameTxt(reg_name).c_str(), reg_value, infos.reg_value_min, infos.reg_value_max);
                }
            }
        }
        else
        {
            TRACE_ERROR(HKX, "[#%i] updateValue(reg %i / %s) [REGISTER ID ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
        }
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] updateValue(reg %i / %s) [REGISTER NAME ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
    }
}

void ServoHerkuleX::commitValue(int reg_name, int commit, int reg_type)
{
    // Find register's informations (addr, size...)
    RegisterInfos infos = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
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

                if (reg_type == REGISTER_AUTO)
                {
                    if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_BOTH;
                    else if (infos.reg_addr_rom >= 0)
                        reg_type = REGISTER_ROM;
                    else if (infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_RAM;
                }

                std::lock_guard <std::mutex> lock(access);

                // Set commit(s)
                if (reg_type == REGISTER_RAM)
                {
                    registerTableCommitsRAM[infos.reg_index] = commit;
                }

                if (reg_type == REGISTER_ROM)
                {
                    registerTableCommits[infos.reg_index] = commit;
                }
            }
            else
            {
                TRACE_ERROR(HKX, "[#%i] commitValue(reg %i / %s) [REGISTER ACCESS ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
            }
        }
        else
        {
            TRACE_ERROR(HKX, "[#%i] commitValue(reg %i / %s) [REGISTER ID ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
        }
    }
    else
    {
        TRACE_ERROR(HKX, "[#%i] commitValue(reg %i / %s) [REGISTER NAME ERROR]", servoId, reg_name, getRegisterNameTxt(reg_name).c_str());
    }
}
