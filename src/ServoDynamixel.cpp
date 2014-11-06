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
 * \file ServoDynamixel.cpp
 * \date 13/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#include "ServoDynamixel.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"
#include "ControlTablesDynamixel.h"

#include <iostream>
#include <string.h>

ServoDynamixel::ServoDynamixel(const int control_table[][8], int dynamixel_id, int dynamixel_model, int speed_mode):
    Servo()
{
    // Store servo id, serie and model
    servoId = dynamixel_id;
    dxl_get_model_infos(dynamixel_model, servoSerie, servoModel);

    // Set the control table
    if (control_table)
    {
        ct = control_table;
    }
    else
    {
        ct = MX_control_table;
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

    // Set model and id because we already known them
    registerTableValues[gid(REG_MODEL_NUMBER)] = dynamixel_model;
    if (dynamixel_id > -1 && dynamixel_id < BROADCAST_ID)
    {
        registerTableValues[gid(REG_ID)] = dynamixel_id;
    }

    // Choose speed mode
    if (speed_mode == SPEED_MANUAL || speed_mode == SPEED_AUTO)
    {
        speedMode = speed_mode;
    }

    // Set default value for ack policy, ACK_REPLY_ALL for Dynamixel devices
    // (will be overwritten when reading the real value from the device)
    registerTableValues[gid(REG_STATUS_RETURN_LEVEL)] = ACK_REPLY_ALL;
}

ServoDynamixel::~ServoDynamixel()
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

void ServoDynamixel::status()
{
    std::lock_guard <std::mutex> lock(access);

    std::cout << "Status(#" << servoId << ")" << std::endl;

    std::cout << "> model      : " << servoModel << std::endl;
    std::cout << "> firmware   : " << registerTableValues[gid(REG_FIRMWARE_VERSION)] << std::endl;
    std::cout << "> baudrate   : " << dxl_get_baudrate(registerTableValues[gid(REG_BAUD_RATE)]) << std::endl;

    std::cout << ">> speed mode     : " << speedMode << std::endl;
    std::cout << ">> steps          : " << steps << std::endl;
    std::cout << ">> runningDegrees : " << runningDegrees << std::endl;

    std::cout << "> torque enabled  : " << registerTableValues[gid(REG_TORQUE_ENABLE)] << std::endl;
    std::cout << "> max torque      : " << registerTableValues[gid(REG_MAX_TORQUE)] << std::endl;
    std::cout << "> torque limit    : " << registerTableValues[gid(REG_TORQUE_LIMIT)] << std::endl;

    std::cout << "> goal position   : " << registerTableValues[gid(REG_GOAL_POSITION)] << std::endl;
    std::cout << "> goal speed      : " << registerTableValues[gid(REG_GOAL_SPEED)] << std::endl;
    std::cout << "> current position: " << registerTableValues[gid(REG_CURRENT_POSITION)] << std::endl;
    std::cout << "> current speed   : " << registerTableValues[gid(REG_CURRENT_SPEED)] << std::endl;
    std::cout << "> current load    : " << registerTableValues[gid(REG_CURRENT_LOAD)] << std::endl;
    std::cout << "> current voltage : " << registerTableValues[gid(REG_CURRENT_VOLTAGE)] << std::endl;
    std::cout << "> current temp    : " << registerTableValues[gid(REG_CURRENT_TEMPERATURE)] << std::endl;
    std::cout << "> registered      : " << registerTableValues[gid(REG_REGISTERED)] << std::endl;
    std::cout << "> moving          : " << registerTableValues[gid(REG_MOVING)] << std::endl;
    std::cout << "> lock            : " << registerTableValues[gid(REG_LOCK)] << std::endl;
    std::cout << "> punch           : " << registerTableValues[gid(REG_PUNCH)] << std::endl;
}

int ServoDynamixel::getSpeedMode()
{
    return speedMode;
}

void ServoDynamixel::setSpeedMode(int speed_mode)
{
    if (speed_mode == SPEED_MANUAL || speed_mode == SPEED_AUTO)
    {
        speedMode = speed_mode;
    }
}

std::string ServoDynamixel::getModelString()
{
    std::lock_guard <std::mutex> lock(access);
    return dxl_get_model_name(registerTableValues[gid(REG_MODEL_NUMBER)]);
}

void ServoDynamixel::getModelInfos(int &servo_serie, int &servo_model)
{
    std::lock_guard <std::mutex> lock(access); // ?
    int model_number = registerTableValues[gid(REG_MODEL_NUMBER)];

    dxl_get_model_infos(model_number, servo_serie, servo_model);
}

/* ************************************************************************** */

int ServoDynamixel::getBaudRate()
{
    std::lock_guard <std::mutex> lock(access);
    return dxl_get_baudrate(registerTableValues[gid(REG_BAUD_RATE)], servoSerie);
}

int ServoDynamixel::getReturnDelay()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_RETURN_DELAY_TIME)];
}

double ServoDynamixel::getHighestLimitTemp()
{
    std::lock_guard <std::mutex> lock(access);
    return static_cast<double>(registerTableValues[gid(REG_TEMPERATURE_LIMIT)]);
}

double ServoDynamixel::getLowestLimitVolt()
{
    std::lock_guard <std::mutex> lock(access);
    int volt = registerTableValues[gid(REG_VOLTAGE_LOWEST_LIMIT)];

    return (volt / 10.0);
}

double ServoDynamixel::getHighestLimitVolt()
{
    std::lock_guard <std::mutex> lock(access);
    int volt = registerTableValues[gid(REG_VOLTAGE_HIGHEST_LIMIT)];

    return (volt / 10.0);
}

int ServoDynamixel::getMaxTorque()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MAX_TORQUE)];
}

int ServoDynamixel::getGoalPosition()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_GOAL_POSITION)];
}

int ServoDynamixel::getMovingSpeed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_GOAL_SPEED)];
}

int ServoDynamixel::getTorqueLimit()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_TORQUE_LIMIT)];
}

int ServoDynamixel::getCurrentPosition()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_POSITION)];
}

int ServoDynamixel::getCurrentSpeed()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_SPEED)];
}

int ServoDynamixel::getCurrentLoad()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_CURRENT_LOAD)];
}

double ServoDynamixel::getCurrentVoltage()
{
    std::lock_guard <std::mutex> lock(access);
    int ivolt = registerTableValues[gid(REG_CURRENT_VOLTAGE)];

    return (ivolt / 10.0);
}

double ServoDynamixel::getCurrentTemperature()
{
    std::lock_guard <std::mutex> lock(access);
    return static_cast<double>(registerTableValues[gid(REG_CURRENT_TEMPERATURE)]);
}

int ServoDynamixel::getRegistered()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_REGISTERED)];
}

int ServoDynamixel::getMoving()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_MOVING)];
}

int ServoDynamixel::getLock()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_LOCK)];
}

int ServoDynamixel::getPunch()
{
    std::lock_guard <std::mutex> lock(access);
    return registerTableValues[gid(REG_PUNCH)];
}

/* ************************************************************************** */

void ServoDynamixel::setId(int id)
{
    //std::cout << "[#" << servoId << "] setId(" << getId() << ", " << id << ")" << std::endl;

    if (id > -1 && id < 254)
    {
        std::lock_guard <std::mutex> lock(access);

        // Maybe check if new ID is not already in use ?
        registerTableValues[gid(REG_ID)] = id;
        registerTableCommits[gid(REG_ID)] = 1;
    }
}

void ServoDynamixel::setCWLimit(int limit)
{
    //std::cout << "[#" << servoId << "] setCWLimit(" << getId() << ", " << limit << ")" << std::endl;

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MIN_POSITION)] = limit;
        registerTableCommits[gid(REG_MIN_POSITION)] = 1;
    }
}

void ServoDynamixel::setCCWLimit(int limit)
{
    //std::cout << "[#" << servoId << "] setCCWLimit(" << getId() << ", " << limit << ")" << std::endl;

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_POSITION)] = limit;
        registerTableCommits[gid(REG_MAX_POSITION)] = 1;
    }
}

void ServoDynamixel::setGoalPosition(int pos)
{
    //std::cout << "[#" << servoId << "] DXL setGoalPosition(" << getId() << ", " << pos << ")" << std::endl;

    if (pos > -1 && pos < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_GOAL_POSITION)] = pos;
        registerTableCommits[gid(REG_GOAL_POSITION)] = 1;
    }
    else
    {
        std::cerr << "[#" << servoId << "] setGoalPosition(" << registerTableValues[gid(REG_CURRENT_POSITION)] << " > " << pos << ") [VALUE ERROR]" << std::endl;
    }
}

void ServoDynamixel::moveGoalPosition(int move)
{
    //std::cout << "moveGoalPosition(" << getId() << ", " << move << ")" << std::endl;
    std::lock_guard <std::mutex> lock(access);

    int curr = registerTableValues[gid(REG_CURRENT_POSITION)];
    int newpos = registerTableValues[gid(REG_CURRENT_POSITION)] + move;

    // Wheel mode ?
    if (registerTableValues[gid(REG_MIN_POSITION)] == 0 && registerTableValues[gid(REG_MAX_POSITION)] == 0)
    {
        if (newpos < 0 || newpos > steps)
        {
            int mod = newpos % steps;

            std::cerr << "[#" << servoId << "]  moveGoalPosition([" << curr << " > " << newpos << "] [VALUE ERROR] with modulo: " << mod <<  std::endl;
        }
    }

    if (newpos > 0 && newpos < steps)
    {
        registerTableValues[gid(REG_GOAL_POSITION)] = registerTableValues[gid(REG_CURRENT_POSITION)] + move;
        registerTableCommits[gid(REG_GOAL_POSITION)] = 1;
    }
    else
    {
        std::cerr << "[#" << servoId << "]  moveGoalPosition([" << curr << " > " << newpos << "] [VALUE ERROR]" <<  std::endl;
    }
}

void ServoDynamixel::setMovingSpeed(int speed)
{
    //std::cout << "setMovingSpeed(" << getId() << ", " << speed << ")" << std::endl;
    std::lock_guard <std::mutex> lock(access);

    if (registerTableValues[gid(REG_MIN_POSITION)] == 0 &&
        registerTableValues[gid(REG_MAX_POSITION)] == 0)
    {
        if (speed < 2048)
        {
            registerTableValues[gid(REG_GOAL_SPEED)] = speed;
            registerTableCommits[gid(REG_GOAL_SPEED)] = 1;
        }
    }
    else
    {
        if (speed < 1024)
        {
            registerTableValues[gid(REG_GOAL_SPEED)] = speed;
            registerTableCommits[gid(REG_GOAL_SPEED)] = 1;
        }
    }
}

void ServoDynamixel::setMaxTorque(int torque)
{
    //std::cout << "[#" << servoId << "] setMaxTorque(" << getId() << ", " << torque << ")" << std::endl;

    if (torque < 1024)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_TORQUE)] = torque;
        registerTableCommits[gid(REG_MAX_TORQUE)] = 1;
    }
}

void ServoDynamixel::setLed(int led)
{
    //std::cout << "[#" << servoId << "] setLed(" << getId() << ", " << led << ")" << std::endl;
    std::lock_guard <std::mutex> lock(access);

    // Normalize value
    if (led >= 1)
    {
        (servoSerie == SERVO_XL) ? led = led : led = 1;
    }
    else
    {
        led = 0;
    }

    registerTableValues[gid(REG_LED)] = led;
    registerTableCommits[gid(REG_LED)] = 1;
}

void ServoDynamixel::setTorqueEnabled(int torque)
{
    //std::cout << "[#" << servoId << "] setTorqueEnabled(" << getId() << ", " << torque << ")" << std::endl;
    std::lock_guard <std::mutex> lock(access);

    // Normalize value
    (torque > 0) ? torque = 1 : torque = 0;

    registerTableValues[gid(REG_TORQUE_ENABLE)] = torque;
    registerTableCommits[gid(REG_TORQUE_ENABLE)] = 1;
}
