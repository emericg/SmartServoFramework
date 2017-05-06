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
 * \file DynamixelController.cpp
 * \date 14/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "DynamixelController.h"
#include "minitraces.h"

// C++ standard libraries
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>

// Enable latency timer
//#define LATENCY_TIMER

DynamixelController::DynamixelController(int ctrlFrequency, int servoSerie):
    ControllerAPI(ctrlFrequency)
{
    this->servoSerie = servoSerie;
}

DynamixelController::~DynamixelController()
{
    disconnect();
}

void DynamixelController::updateInternalSettings()
{
    if (servoSerie != SERVO_UNKNOWN)
    {
        if (servoSerie >= SERVO_HERKULEX)
        {
            ackPolicy = 1;
            maxId = 253;

            protocolVersion = PROTOCOL_HKX;
            TRACE_INFO(CAPI, "- Using HerkuleX communication protocol");
        }
        else if (servoSerie >= SENSOR_DYNAMIXEL)
        {
            // TODO
        }
        else if (servoSerie >= SERVO_DYNAMIXEL)
        {
            ackPolicy = 2;
            maxId = 252;

            if (servoSerie >= SERVO_XL)
            {
                protocolVersion = PROTOCOL_DXLv2;
            }
            else // SERVO AX to MX
            {
                protocolVersion = PROTOCOL_DXLv1;

                if (serialDevice == SERIAL_USB2AX)
                {
                    // The USB2AX adapter reserves the ID 253 for itself
                    maxId = 252;
                }
                else
                {
                    maxId = 253;
                }
            }

            if (protocolVersion == PROTOCOL_DXLv2)
            {
                TRACE_INFO(CAPI, "- Using Dynamixel communication protocol version 2");
            }
            else
            {
                TRACE_INFO(CAPI, "- Using Dynamixel communication protocol version 1");
            }
        }
    }
    else
    {
        TRACE_WARNING(CAPI, "Warning: Unknown servo serie!");
    }
}

void DynamixelController::changeProtocolVersion(int protocol)
{
    if (protocol != protocolVersion)
    {
        if (protocol == PROTOCOL_DXLv1)
        {
            protocolVersion = PROTOCOL_DXLv1;
            if (serialDevice == SERIAL_USB2AX)
            {
                // The USB2AX device uses the ID 253 for itself
                maxId = 252;
            }
            else
            {
                maxId = 253;
            }
            TRACE_INFO(CAPI, "- Using Dynamixel communication protocol version 1");
        }
        else if (protocol == PROTOCOL_DXLv2)
        {
            protocolVersion = PROTOCOL_DXLv2;
            maxId = 252;
            TRACE_INFO(CAPI, "- Using Dynamixel communication protocol version 2");
        }
        else
        {
            TRACE_ERROR(CAPI, "- Unknown Dynamixel communication protocol (version %i), unable to use it!", protocol);
        }
    }
}

int DynamixelController::connect(std::string &devicePath, const int baud, const int serialDevice)
{
    // Make sure the serial link is not already connected
    disconnect();

    // Update device infos
    this->serialDevice = serialDevice;
    updateInternalSettings();

    // Connection
    int retcode = serialInitialize(devicePath, baud);

    if (retcode == 1)
    {
        startThread();
    }

    return retcode;
}

void DynamixelController::disconnect()
{
    stopThread();
    serialTerminate();
}

std::string DynamixelController::serialGetCurrentDevice_wrapper()
{
    return serialGetCurrentDevice();
}

std::vector <std::string> DynamixelController::serialGetAvailableDevices_wrapper()
{
    return serialGetAvailableDevices();
}

void DynamixelController::serialSetLatency_wrapper(int latency)
{
    serialSetLatency(latency);
}

void DynamixelController::autodetect_internal(int start, int stop)
{
    setState(state_scanning);

    // Prepare to scan, clean servoLists
    unregisterServos_internal();

    // Check start/stop boundaries
    if (start < 0 || start > (maxId - 1))
        start = 0;

    if (stop < 1 || stop > maxId || stop < start)
        stop = maxId;

#if defined(_WIN32) || defined(_WIN64)
    // Bring RX packet timeout down to scan faster
    serialSetLatency(12);
#else
    // Bring RX packet timeout down to scan way faster
    serialSetLatency(8);
#endif

    TRACE_INFO(CAPI, "DXL ctrl_device_autodetect(port: '%s' / tid: '%i')",
               serialGetCurrentDevice().c_str(), std::this_thread::get_id());

    TRACE_INFO(CAPI, "> THREADED Scanning for DXL devices on '%s', protocol v%i, range is [%i,%i[",
               serialGetCurrentDevice().c_str(), protocolVersion, start, stop);

    for (int id = start; id <= stop; id++)
    {
        PingResponse pingstats;

        // If the ping gets a response, then we have found a servo
        if (dxl_ping(id, &pingstats) == true)
        {
            //setLed(id, 1, LED_GREEN);

            int serie, model;
            dxl_get_model_infos(pingstats.model_number, serie, model);
            ServoDynamixel *servo = nullptr;

            TRACE_INFO(DXL, "[#%i] %s servo found!", id, dxl_get_model_name(pingstats.model_number).c_str());

            // Instanciate the device found
            switch (serie)
            {
            case SERVO_AX:
            case SERVO_DX:
            case SERVO_RX:
                servo = new ServoAX(id, pingstats.model_number);
                break;

            case SERVO_EX:
                servo = new ServoEX(id, pingstats.model_number);
                break;

            case SERVO_MX:
                servo = new ServoMX(id, pingstats.model_number);
                break;

            case SERVO_XL:
                servo = new ServoXL(id, pingstats.model_number);
                break;

            case SERVO_X:
                servo = new ServoX(id, pingstats.model_number);
                break;

            default:
                break;
            }

            if (servo != nullptr)
            {
                servoListLock.lock();

                // Add the servo to the controller
                servoList.push_back(servo);

                // Mark it for an "initial read" and synchronization
                updateList.push_back(servo->getId());
                syncList.push_back(servo->getId());

                servoListLock.unlock();
            }

            //setLed(id, 0);
        }
        else
        {
            printf(".");
        }
    }

    printf("\n");

    // Restore RX packet timeout
    serialSetLatency(LATENCY_TIME_DEFAULT);

    if (getState() >= state_started)
    {
        setState(state_scanned);
    }
}

void DynamixelController::run()
{
    TRACE_INFO(CAPI, "DynamixelController::run(port: '%s' / tid: '%i')",
               serialGetCurrentDevice().c_str(), std::this_thread::get_id());

    std::chrono::time_point<std::chrono::system_clock> start, end;

    while (getState() >= state_started)
    {
        // Loop timer
        start = std::chrono::system_clock::now();

        // MESSAGE PARSING
        ////////////////////////////////////////////////////////////////////////

        unsigned ignoreMsg = 0;

        m_mutex.lock();
        while (m_queue.empty() == false && m_queue.size() != ignoreMsg)
        {
            miniMessages m = m_queue.front();
            m_mutex.unlock();

            switch (m.msg)
            {
            case ctrl_device_autodetect:
                autodetect_internal(m.p1, m.p2);
                break;

            case ctrl_device_register:
                // handle registering servo by "id"?
                registerServo_internal(static_cast<Servo *>(m.p));
                break;
            case ctrl_device_unregister:
                unregisterServo_internal(static_cast<Servo *>(m.p));
                break;
            case ctrl_device_unregister_all:
                unregisterServos_internal();
                break;

            case ctrl_device_delayed_add:
                if (delayedAddServos_internal(m.delay, m.p1, m.p2) == 1)
                {
                    ignoreMsg++;
                    sendMessage(&m);
                }
                break;

            case ctrl_state_pause:
                TRACE_INFO(CAPI, ">> THREAD (tid: '%i') paused by message", std::this_thread::get_id());
                m_mutex.lock();
                m_queue.pop_front();
                m_mutex.unlock();
                return;
                break;
            case ctrl_state_stop:
                TRACE_INFO(CAPI, ">> THREAD (tid: '%i') termination by 'stop message'", std::this_thread::get_id());
                m_mutex.lock();
                m_queue.pop_front();
                m_mutex.unlock();
                return;
                break;

            default:
                TRACE_WARNING(DXL, "Unknown message type: '%i'", m.msg);
                break;
            }

            m_mutex.lock();
            m_queue.pop_front();
        }
        m_mutex.unlock();

        // ACTION LOOP
        ////////////////////////////////////////////////////////////////////////

        servoListLock.lock();
        for (auto s: servoList)
        {
            int id = s->getId();
            int ack = s->getStatusReturnLevel();

            int actionProgrammed, rebootProgrammed, refreshProgrammed, resetProgrammed;
            s->getActions(actionProgrammed, rebootProgrammed, refreshProgrammed, resetProgrammed);

            if (refreshProgrammed == 1)
            {
                // Every servo register value will be updated
                updateList.push_back(id);
                TRACE_INFO(DXL, "Refresh servo #%i registers", id);
            }

            if (actionProgrammed == 1)
            {
                dxl_action(id, ack);
                TRACE_INFO(DXL, "Action for servo #%i", id);
            }

            if (rebootProgrammed == 1)
            {
                // Remove servo from sync/update lists; Need to be added again after reboot!
                for (std::vector <int>::iterator it = updateList.begin(); it != updateList.end();)
                {
                    if (*it == id)
                    { updateList.erase(it); }
                    else
                    { ++it; }
                }
                for (std::vector <int>::iterator it = syncList.begin(); it != syncList.end();)
                {
                    if (*it == id)
                    { syncList.erase(it); }
                    else
                    { ++it; }
                }

                // Reboot
                dxl_reboot(id, ack);
                TRACE_INFO(DXL, "Rebooting servo #%i...", id);

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), nullptr, id, 0};
                sendMessage(&m);
            }

            if (resetProgrammed > 0)
            {
                // Remove servo from sync/update lists; Need to be added again after reset!
                for (std::vector <int>::iterator it = updateList.begin(); it != updateList.end();)
                {
                    if (*it == id)
                    { updateList.erase(it); }
                    else
                    { ++it; }
                }
                for (std::vector <int>::iterator it = syncList.begin(); it != syncList.end();)
                {
                    if (*it == id)
                    { syncList.erase(it); }
                    else
                    { ++it; }
                }

                // Reset
                dxl_reset(id, resetProgrammed, ack);
                TRACE_INFO(DXL, "Resetting servo #%i (setting: %i)...", id, resetProgrammed);

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), nullptr, id, 1};
                sendMessage(&m);
            }
        }
        servoListLock.unlock();

        // INITIAL READ LOOP
        ////////////////////////////////////////////////////////////////////////

        servoListLock.lock();
        if (updateList.empty() == false)
        {
            std::vector <int>::iterator itr;
            for (itr = updateList.begin(); itr != updateList.end();)
            {
                setState(state_reading);

                for (auto s: servoList)
                {
                    if (s->getId() == (*itr))
                    {
                        int id = s->getId();
                        int ack = s->getStatusReturnLevel();

                        for (int ctid = 1; ctid < s->getRegisterCount(); ctid++)
                        {
                            int reg_name = getRegisterName(s->getControlTable(), ctid);
                            int reg_addr = getRegisterAddr(s->getControlTable(), reg_name);
                            int reg_size = getRegisterSize(s->getControlTable(), reg_name);

                            TRACE_1(DXL, "Reading value for reg [%i] name: '%s' addr: '%i' size: '%i'", ctid, getRegisterNameTxt(reg_name).c_str(), reg_addr, reg_size);

                            if (reg_size == 1)
                            {
                                s->updateValue(reg_name, dxl_read_byte(id, reg_addr, ack));
                            }
                            else //if (regsize == 2)
                            {
                                s->updateValue(reg_name, dxl_read_word(id, reg_addr, ack));
                            }
                            s->setError(dxl_get_rxpacket_error());
                            updateErrorCount(dxl_get_com_error_count());
                            dxl_print_error();
                        }

                        // Once all registers are read, remove the servo from the "updateList"
                        itr = updateList.erase(itr);
                    }
                }
            }

            setState(state_ready);
        }
        servoListLock.unlock();

        // SYNCHRONIZATION LOOP
        ////////////////////////////////////////////////////////////////////////

        int cumulid = 0;

        servoListLock.lock();
        for (auto id: syncList)
        {
            cumulid++;
            cumulid %= syncloopFrequency;

            for (auto s_raw: servoList)
            {
                servoListLock.unlock();

                ServoDynamixel *s = static_cast<ServoDynamixel*>(s_raw);

                if (s->getId() == id)
                {
                    int ack = s->getStatusReturnLevel();

                    // Unregister device if it reach an error count too high
                    // Count must be high enough to avoid "false positive": device producing a lot of errors but still present on the serial link
                    if (s->getErrorCount() > 16)
                    {
                        TRACE_ERROR(DXL, "Device #%i has an error count too high and is going to be unregistered from its controller on '%s'...", id, serialGetCurrentDevice().c_str());
                        unregisterServo(s);
                        continue;
                    }

                    // Commit register modifications
                    for (int ctid = 0; ctid < s->getRegisterCount(); ctid++)
                    {
                        int reg_name = getRegisterName(s->getControlTable(), ctid);

                        if (s->getValueCommit(reg_name) == 1)
                        {
                            int reg_addr = getRegisterAddr(s->getControlTable(), reg_name);
                            int reg_size = getRegisterSize(s->getControlTable(), reg_name);

                            if ((s->getSpeedMode() == SPEED_AUTO && (reg_name != REG_GOAL_POSITION && reg_name != REG_GOAL_SPEED)) == false)
                            {
                                TRACE_1(DXL, "Writing value '%i' for reg [%i] name: '%s' addr: '%i' size: '%i'",
                                        s->getValue(reg_name), ctid, getRegisterNameTxt(reg_name).c_str(), reg_addr, reg_size);

                                if (reg_size == 1)
                                {
                                    dxl_write_byte(id, reg_addr, s->getValue(reg_name), ack);
                                }
                                else //if (regsize == 2)
                                {
                                    dxl_write_word(id, reg_addr, s->getValue(reg_name), ack);
                                }

                                s->commitValue(reg_name, 0);
                                s->setError(dxl_get_rxpacket_error());
                                updateErrorCount(dxl_get_com_error_count());
                                dxl_print_error();

                                if (reg_name == REG_ID)
                                {
                                    if (s->changeInternalId(s->getValue(reg_name)) == 1)
                                    {
                                        s->reboot();
                                    }
                                }
                            }
                        }
                    }

                    // 1 Hz "low priority" update loop
                    if (((syncloopCounter - cumulid) == 0) &&
                        (ack != ACK_NO_REPLY))
                    {
                        // Read voltage
                        s->updateValue(REG_CURRENT_VOLTAGE, dxl_read_byte(id, s->gaddr(REG_CURRENT_VOLTAGE), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();

                        // Read temp
                        s->updateValue(REG_CURRENT_TEMPERATURE, dxl_read_byte(id, s->gaddr(REG_CURRENT_TEMPERATURE), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();
                    }

                    // x/4 Hz "feedback" update loop
                    if (((syncloopCounter - cumulid) % 4 == 0) &&
                        (ack != ACK_NO_REPLY))
                    {
                        s->updateValue(REG_CURRENT_SPEED, dxl_read_word(id, s->gaddr(REG_CURRENT_SPEED), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();

                        s->updateValue(REG_CURRENT_LOAD, dxl_read_word(id, s->gaddr(REG_CURRENT_LOAD), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();

                        // Read moving
                        s->updateValue(REG_MOVING, dxl_read_byte(id, s->gaddr(REG_MOVING), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();
                    }

                    // x Hz "full speed" update loop
                    {
                        // Get "current" values from devices, and write them into corresponding objects
                        int cpos = dxl_read_word(id, s->gaddr(REG_CURRENT_POSITION), ack);
                        s->updateValue(REG_CURRENT_POSITION, cpos);
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error_count());
                        dxl_print_error();

                        // Goal pos
                        if (s->getValueCommit(REG_GOAL_POSITION) == 1)
                        {
                            int gpos = s->getGoalPosition();
                            int movingSpeed = 50; //s->getMovingSpeed();

                            // Control modes:
                            if (s->getSpeedMode() == SPEED_AUTO)
                            {
                                double k = 1.0; // acceleration factor
                                double mot = 3.0; // margin of tolerance

                                if (s->getCwAngleLimit() != 0 || s->getCcwAngleLimit() != 0) // JOINT MODE
                                {
                                    double step = static_cast<double>(s->getRunningDegrees()) / s->getSteps();
                                    double angle = static_cast<double>(gpos - cpos) * step;
                                    double angle_abs = std::fabs(angle);
                                    int speed = (movingSpeed + static_cast<int>(k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        // SPEED
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                        updateErrorCount(dxl_get_com_error_count());
                                        dxl_print_error();
                                        s->setError(dxl_get_rxpacket_error());

                                        // POS
                                        if (angle >= 0)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getSteps() - 1, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), 0, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }

                                        TRACE_2(DXL, "pos: '%i' Movingspeed: '%i' CurrentSpeed: '%i'   |   (> %i) (angle: %i)",
                                                cpos, speed, s->getCurrentSpeed(), gpos, angle);
                                    }
                                    else // STOP
                                    {
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), movingSpeed, ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error_count());
                                        dxl_print_error();

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition(), ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error_count());
                                        dxl_print_error();

                                        TRACE_2(DXL, "[STOP] pos: '%i' speed: '%i'   |   (> %i) (angle: %i)",
                                                cpos, speed, gpos, angle);
                                        s->commitValue(REG_GOAL_POSITION, 0);
                                    }
                                }
                                else // if (s->getCwAngleLimit() == 0 && s->getCcwAngleLimit() == 0) // WHEEL MODE
                                {
                                    double step = 360.0 / s->getSteps();
                                    double angle = static_cast<double>(gpos - cpos) * step;

                                    if (angle > 180) angle -= 360;
                                    else if (angle < -180) angle += 360;
                                    double angle_abs = std::fabs(angle);

                                    int speed = (movingSpeed + static_cast<int>(k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        if (angle >= 0)
                                        {
                                            // SPEED (counter clockwise)
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            // SPEED (clockwise)
                                            speed +=  1024;
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }

                                        TRACE_2(DXL, "pos: '%i' Movingspeed: '%i' CurrentSpeed: '%i'   |   (> %i) (angle: %i)",
                                                cpos, speed, s->getCurrentSpeed(), gpos, angle);
                                    }
                                    else // STOP
                                    {
                                        if (dxl_read_word(id, s->gaddr(REG_GOAL_SPEED), ack) >= 1024)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), ack, 1024);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), 0, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error_count());
                                            dxl_print_error();
                                        }

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition(), ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error_count());
                                        dxl_print_error();

                                        TRACE_2(DXL, "[STOP] pos: '%i' speed: '%i'   |   (> %i) (angle: %i)",
                                                cpos, speed, gpos, angle);
                                        s->commitValue(REG_GOAL_POSITION, 0);
                                    }
                                }
                            }
                            else if (s->getSpeedMode() == SPEED_MANUAL)
                            {
                                if (s->getCwAngleLimit() == 0 || s->getCcwAngleLimit() == 0) // WHEEL MODE
                                {
                                    // WIP // Do we want to handle this on the framework side ?
                                }
                            }
                        }
                    }
                }

                servoListLock.lock();
            }
        }

        // Make sure we unlock servoList
        servoListLock.unlock();

        // Loop control
        syncloopCounter++;
        syncloopCounter %= syncloopFrequency;

        // Loop timer
        end = std::chrono::system_clock::now();
        double loopd = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        double waitd = (syncloopDuration * 1000.0) - loopd;

#ifdef LATENCY_TIMER
        if ((loopd / 1000.0) > syncloopDuration)
        {
            TRACE_WARNING(DXL, "Sync loop duration: %fms of the %fms budget.", (loopd / 1000.0), syncloopDuration);
        }
        else
        {
            TRACE_INFO(DXL, "Sync loop duration: %fms of the %fms budget.", (loopd / 1000.0), syncloopDuration);
        }
#endif

        if (waitd > 0.0)
        {
            std::chrono::microseconds waittime(static_cast<int>(waitd));
            std::this_thread::sleep_for(waittime);
        }
    }

    TRACE_INFO(DXL, ">> THREAD (tid: '%i') termination by 'loop exit'", std::this_thread::get_id());
}
