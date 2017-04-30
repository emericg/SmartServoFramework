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
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ServoDynamixel.h"
#include "minitraces.h"

#include "Dynamixel.h"
#include "DynamixelTools.h"
#include "ControlTablesDynamixel.h"

#include <thread>
#include <cstring>

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
}

/* ************************************************************************** */

void ServoDynamixel::status()
{
    std::lock_guard <std::mutex> lock(access);

    TRACE_INFO(DXL, "Status(#%i)", servoId);

    TRACE_INFO(DXL, "> model      : %i", servoModel);
    TRACE_INFO(DXL, "> firmware   : %i", registerTableValues[gid(REG_FIRMWARE_VERSION)]);
    TRACE_INFO(DXL, "> baudrate   : %i", dxl_get_baudrate(registerTableValues[gid(REG_BAUD_RATE)]));

    TRACE_INFO(DXL, ">> speed mode     : %i", speedMode);
    TRACE_INFO(DXL, ">> steps          : %i", steps);
    TRACE_INFO(DXL, ">> runningDegrees : %i", runningDegrees);

    TRACE_INFO(DXL, "> torque enabled  : %i", registerTableValues[gid(REG_TORQUE_ENABLE)]);
    TRACE_INFO(DXL, "> max torque      : %i", registerTableValues[gid(REG_MAX_TORQUE)]);
    TRACE_INFO(DXL, "> torque limit    : %i", registerTableValues[gid(REG_TORQUE_LIMIT)]);

    TRACE_INFO(DXL, "> goal position   : %i", registerTableValues[gid(REG_GOAL_POSITION)]);
    TRACE_INFO(DXL, "> goal speed      : %i", registerTableValues[gid(REG_GOAL_SPEED)]);
    TRACE_INFO(DXL, "> current position: %i", registerTableValues[gid(REG_CURRENT_POSITION)]);
    TRACE_INFO(DXL, "> current speed   : %i", registerTableValues[gid(REG_CURRENT_SPEED)]);
    TRACE_INFO(DXL, "> current load    : %i", registerTableValues[gid(REG_CURRENT_LOAD)]);
    TRACE_INFO(DXL, "> current voltage : %i", registerTableValues[gid(REG_CURRENT_VOLTAGE)]);
    TRACE_INFO(DXL, "> current temp    : %i", registerTableValues[gid(REG_CURRENT_TEMPERATURE)]);
    TRACE_INFO(DXL, "> registered      : %i", registerTableValues[gid(REG_REGISTERED)]);
    TRACE_INFO(DXL, "> moving          : %i", registerTableValues[gid(REG_MOVING)]);
    TRACE_INFO(DXL, "> lock            : %i", registerTableValues[gid(REG_LOCK)]);
    TRACE_INFO(DXL, "> punch           : %i", registerTableValues[gid(REG_PUNCH)]);
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

void ServoDynamixel::waitMovementCompletion(int timeout_ms)
{
    std::chrono::milliseconds timeout_duration(static_cast<int>(timeout_ms));
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    access.lock();

    int c = registerTableValues[gid(REG_CURRENT_POSITION)];
    int g = registerTableValues[gid(REG_GOAL_POSITION)];

    // Margin is set to 3% of servo steps
    int margin = static_cast<int>(static_cast<double>(steps) * 0.03 / 2.0);
    int margin_up = g + margin;
    int margin_dw = g - margin;

    if (margin_up > steps) margin_up = steps;
    if (margin_dw < 0) margin_dw = 0;

    // Wait until the current pos is within margin of the goal pos, or wait for the timeout
    while (!(c < margin_up && c > margin_dw))
    {
        access.unlock();

        TRACE_2(DXL, "waitMovementCompletion(%i < pos: %i < %i)", margin_dw, c, margin_up);

        if ((start + timeout_duration) < std::chrono::system_clock::now())
        {
            TRACE_WARNING(DXL, "waitMovementCompletion() timeout!", margin_dw, c, margin_up);
            return;
        }

        std::chrono::milliseconds loopwait(4);
        std::this_thread::sleep_for(loopwait);

        access.lock();

        c = registerTableValues[gid(REG_CURRENT_POSITION)];
        g = registerTableValues[gid(REG_GOAL_POSITION)];
    }
    access.unlock();
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
    TRACE_1(DXL, "[#%i] setId(from %i to %i)", servoId, servoId, id);

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
    TRACE_1(DXL, "[#%i] setCWLimit(%i)", servoId, limit);

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MIN_POSITION)] = limit;
        registerTableCommits[gid(REG_MIN_POSITION)] = 1;
    }
}

void ServoDynamixel::setCCWLimit(int limit)
{
    TRACE_1(DXL, "[#%i] setCCWLimit(%i)", servoId, limit);

    if (limit > -1 && limit < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_POSITION)] = limit;
        registerTableCommits[gid(REG_MAX_POSITION)] = 1;
    }
}

void ServoDynamixel::setGoalPosition(int pos)
{
    TRACE_1(DXL, "[#%i] setGoalPosition(%i)", servoId, pos);

    if (pos > -1 && pos < steps)
    {
        std::lock_guard <std::mutex> lock(access);

        // Check min/max positions
        if (pos < registerTableValues[gid(REG_MIN_POSITION)])
        {
            pos = registerTableValues[gid(REG_MIN_POSITION)];
        }
        if (pos > registerTableValues[gid(REG_MAX_POSITION)])
        {
            pos = registerTableValues[gid(REG_MAX_POSITION)];
        }

        // Set position
        registerTableValues[gid(REG_GOAL_POSITION)] = pos;
        registerTableCommits[gid(REG_GOAL_POSITION)] = 1;
    }
    else
    {
        TRACE_ERROR(DXL, "[#%i] setGoalPosition(%i > %i) [VALUE ERROR]", servoId, registerTableValues[gid(REG_CURRENT_POSITION)], pos);
    }
}

void ServoDynamixel::setGoalPosition(int pos, int time_budget_ms)
{
    TRACE_1(DXL, "[#%i] setGoalPosition(%i in %ims)", servoId, pos, time_budget_ms);

    if (time_budget_ms > 0)
    {
        if (pos > -1 && pos < steps)
        {
            std::lock_guard <std::mutex> lock(access);

            // Check min/max positions
            if (pos < registerTableValues[gid(REG_MIN_POSITION)])
            {
                pos = registerTableValues[gid(REG_MIN_POSITION)];
            }
            if (pos > registerTableValues[gid(REG_MAX_POSITION)])
            {
                pos = registerTableValues[gid(REG_MAX_POSITION)];
            }

            // Compute movment amplitude and necessary speed to meet time budget
            int amp = 0;
            int pos_curr = registerTableValues[gid(REG_CURRENT_POSITION)];

            if (pos > pos_curr)
                amp = pos - pos_curr;
            else
                amp = pos_curr - pos;

            double speed = ((double)(60 * amp) / (double)(steps * 0.114 * ((double)(time_budget_ms)/1000.0)));
            if (speed < 1.0)
                speed = 1;

            // Set position and speed
            registerTableValues[gid(REG_GOAL_POSITION)] = pos;
            registerTableCommits[gid(REG_GOAL_POSITION)] = 1;

            registerTableValues[gid(REG_GOAL_SPEED)] = speed;
            registerTableCommits[gid(REG_GOAL_SPEED)] = 1;
        }
        else
        {
            TRACE_ERROR(DXL, "[#%i] setGoalPosition(%i > %i) [VALUE ERROR]", servoId, registerTableValues[gid(REG_CURRENT_POSITION)], pos);
        }
    }
}

void ServoDynamixel::moveGoalPosition(int move)
{
    TRACE_1(DXL, "[#%i] moveGoalPosition(%i)", servoId, move);

    access.lock();

    int curr = registerTableValues[gid(REG_CURRENT_POSITION)];
    int newpos = registerTableValues[gid(REG_CURRENT_POSITION)] + move;

    // Wheel mode ?
    if (registerTableValues[gid(REG_MIN_POSITION)] == 0 && registerTableValues[gid(REG_MAX_POSITION)] == 0)
    {
        if (newpos < 0 || newpos > steps)
        {
            int mod = newpos % steps;

            TRACE_ERROR(DXL, "[#%i]  moveGoalPosition([%i > %i]) [VALUE ERROR] with modulo: %i", servoId, curr, newpos, mod);
        }
    }

    access.unlock();

    setGoalPosition(newpos);
}

void ServoDynamixel::setMovingSpeed(int speed)
{
    TRACE_1(DXL, "[#%i] setMovingSpeed(%i)", servoId, speed);

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
    TRACE_1(DXL, "[#%i] setMaxTorque(%i)", servoId, torque);

    if (torque < 1024)
    {
        std::lock_guard <std::mutex> lock(access);

        registerTableValues[gid(REG_MAX_TORQUE)] = torque;
        registerTableCommits[gid(REG_MAX_TORQUE)] = 1;
    }
}

void ServoDynamixel::setLed(int led)
{
    TRACE_1(DXL, "[#%i] setLed(%i)", servoId, led);

    // Normalize value
    if (led >= 1)
    {
        (servoSerie == SERVO_XL) ? led = led : led = 1;
    }
    else
    {
        led = 0;
    }

    std::lock_guard <std::mutex> lock(access);

    registerTableValues[gid(REG_LED)] = led;
    registerTableCommits[gid(REG_LED)] = 1;
}

void ServoDynamixel::setTorqueEnabled(int torque)
{
    TRACE_1(DXL, "[#%i] setTorqueEnabled(%i)", servoId, torque);

    // Normalize value
    (torque > 0) ? torque = 1 : torque = 0;

    std::lock_guard <std::mutex> lock(access);

    registerTableValues[gid(REG_TORQUE_ENABLE)] = torque;
    registerTableCommits[gid(REG_TORQUE_ENABLE)] = 1;
}
