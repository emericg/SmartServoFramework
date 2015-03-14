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

// C++ standard libraries
#include <iostream>
#include <chrono>
#include <thread>

// Enable latency timer
//#define LATENCY_TIMER

DynamixelController::DynamixelController(int freq, int servoSerie):
    ControllerAPI(freq)
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
        std::cout << std::endl;

        if (servoSerie >= SERVO_HERKULEX)
        {
            ackPolicy = 1;
            maxId = 253;

            protocolVersion = 1;
            std::cout << "- Using HerkuleX communication protocol" << std::endl;
        }
        else if (servoSerie >= SERVO_DYNAMIXEL)
        {
            ackPolicy = 2;
            maxId = 252;

            if (servoSerie >= SERVO_XL)
            {
                protocolVersion = 2;
            }
            else // SERVO AX to MX
            {
                protocolVersion = 1;

                if (serialDevice == SERIAL_USB2AX)
                {
                    // The USB2AX device uses the ID 253 for itself
                    maxId = 252;
                }
                else
                {
                    maxId = 253;
                }
            }

            if (protocolVersion == 2)
            {
                std::cout << "- Using Dynamixel communication protocol version 2" << std::endl;
            }
            else
            {
                std::cout << "- Using Dynamixel communication protocol version 1" << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Warning: Unknown servo serie!" << std::endl;
    }
}

void DynamixelController::changeProtocolVersion(int protocol)
{
    if (protocol != protocolVersion)
    {
        if (protocol == 1)
        {
            protocolVersion = 1;
            if (serialDevice == SERIAL_USB2AX)
            {
                // The USB2AX device uses the ID 253 for itself
                maxId = 252;
            }
            else
            {
                maxId = 253;
            }
            std::cout << "- Using Dynamixel communication protocol version 1" << std::endl;
        }
        else if (protocol == 2)
        {
            protocolVersion = 2;
            maxId = 252;
            std::cout << "- Using Dynamixel communication protocol version 2" << std::endl;
        }
        else
        {
            std::cout << "- Unknown Dynamixel communication protocol (version '" << protocol << "'), unable to use it!" << std::endl;
        }
    }
}

int DynamixelController::connect(std::string &devicePath, const int baud, const int serialDevice)
{
    this->serialDevice = serialDevice;

    updateInternalSettings();

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

    unregisterServos_internal();
    clearMessageQueue();

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

void DynamixelController::serialLockInterface_wrapper()
{
    serialLockInterface();
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

    std::cout << "DXL ctrl_device_autodetect(port: '" << serialGetCurrentDevice() << "' | tid: '" << std::this_thread::get_id() << "')" << std::endl;
    std::cout << "> THREADED Scanning for DXL devices on '" << serialGetCurrentDevice() << "', Dynamixel protocol v" << protocolVersion
              << ", Range is [" << start << "," << stop << "[" << std::endl;

    for (int id = start; id <= stop; id++)
    {
        PingResponse pingstats;

        // If the ping gets a response, then we have found a servo
        if (dxl_ping(id, &pingstats) == true)
        {
            //setLed(id, 1, LED_GREEN);

            int serie, model;
            dxl_get_model_infos(pingstats.model_number, serie, model);
            ServoDynamixel *servo = NULL;

            std::cout << std::endl << "[#" << id << "] " << dxl_get_model_name(pingstats.model_number) << " servo found! ";

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

            default:
                break;
            }

            if (servo != NULL)
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
            std::cout << ".";
        }
    }

    std::cout << std::endl;

    // Restore RX packet timeout
    serialSetLatency(LATENCY_TIME_DEFAULT);

    if (getState() >= state_started)
    {
        setState(state_scanned);
    }
}

void DynamixelController::run()
{
    std::cout << "DynamixelController::run(port: '" << serialGetCurrentDevice() << "' | tid: '" << std::this_thread::get_id() << "')" << std::endl;
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
                std::cout << ">> THREAD (tid: " << std::this_thread::get_id() << "') pause by message" << std::endl;
                m_mutex.lock();
                m_queue.pop_front();
                m_mutex.unlock();
                return;
                break;
            case ctrl_state_stop:
                std::cout << ">> THREAD (tid: " << std::this_thread::get_id() << "') termination by 'stop message'" << std::endl;
                m_mutex.lock();
                m_queue.pop_front();
                m_mutex.unlock();
                return;
                break;

            default:
                std::cerr << "[DXL] Unknown message type: '" << m.msg << "'" << std::endl;
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
                std::cout << "Refresh servo #" << id << " registers"<< std::endl;
            }

            if (actionProgrammed == 1)
            {
                dxl_action(id, ack);
                std::cout << "Action for servo #" << id << std::endl;
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
                std::cout << "Rebooting servo #" << id << "..." << std::endl;

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), NULL, id, 0};
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
                std::cout << "Resetting servo #" << id << " (setting: " << resetProgrammed << ")..." << std::endl;

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), NULL, id, 1};
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
                            int regname = getRegisterName(s->getControlTable(), ctid);
                            int regaddr = getRegisterAddr(s->getControlTable(), regname);
                            int regsize = getRegisterSize(s->getControlTable(), regname);

                            //std::cout << "Reading value for reg [" << ctid << "] name: '" << getRegisterNameTxt(regname)
                            //          << "' addr: '" << regaddr << "' size: '" << regsize << "'" << std::endl;

                            if (regsize == 1)
                            {
                                s->updateValue(regname, dxl_read_byte(id, regaddr, ack));
                            }
                            else //if (regsize == 2)
                            {
                                s->updateValue(regname, dxl_read_word(id, regaddr, ack));
                            }
                            s->setError(dxl_get_rxpacket_error());
                            updateErrorCount(dxl_get_com_error());
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
                        std::cerr << "Device #" << id << " has an error count too high and is going to be unregistered from its controller on '" << serialGetCurrentDevice() << "'..." << std::endl;
                        unregisterServo(s);
                        continue;
                    }

                    // Commit register modifications
                    for (int ctid = 0; ctid < s->getRegisterCount(); ctid++)
                    {
                        int regname = getRegisterName(s->getControlTable(), ctid);

                        if (s->getValueCommit(regname) == 1)
                        {
                            int regaddr = getRegisterAddr(s->getControlTable(), regname);
                            int regsize = getRegisterSize(s->getControlTable(), regname);

                            if ((s->getSpeedMode() == SPEED_AUTO && (regname != REG_GOAL_POSITION && regname != REG_GOAL_SPEED)) == false)
                            {
                                //std::cout << "Writing value '" << s->getValue(regname) << "' for reg [" << ctid << "] name: '" << getRegisterNameTxt(regname)
                                //          << "' addr: '" << regaddr << "' size: '" << regsize << "'" << std::endl;

                                if (regsize == 1)
                                {
                                    dxl_write_byte(id, regaddr, s->getValue(regname), ack);
                                }
                                else //if (regsize == 2)
                                {
                                    dxl_write_word(id, regaddr, s->getValue(regname), ack);
                                }

                                s->commitValue(regname, 0);
                                s->setError(dxl_get_rxpacket_error());
                                updateErrorCount(dxl_get_com_error());
                                dxl_print_error();

                                if (regname == REG_ID)
                                {
                                    if (s->changeInternalId(s->getValue(regname)) == 1)
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
                        updateErrorCount(dxl_get_com_error());
                        dxl_print_error();

                        // Read temp
                        s->updateValue(REG_CURRENT_TEMPERATURE, dxl_read_byte(id, s->gaddr(REG_CURRENT_TEMPERATURE), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error());
                        dxl_print_error();
                    }

                    // x/4 Hz "feedback" update loop
                    if (((syncloopCounter - cumulid) % 4 == 0) &&
                        (ack != ACK_NO_REPLY))
                    {
                        s->updateValue(REG_CURRENT_SPEED, dxl_read_word(id, s->gaddr(REG_CURRENT_SPEED), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error());
                        dxl_print_error();

                        s->updateValue(REG_CURRENT_LOAD, dxl_read_word(id, s->gaddr(REG_CURRENT_LOAD), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error());
                        dxl_print_error();

                        // Read moving
                        s->updateValue(REG_MOVING, dxl_read_byte(id, s->gaddr(REG_MOVING), ack));
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error());
                        dxl_print_error();
                    }

                    // x Hz "full speed" update loop
                    {
                        // Get "current" values from devices, and write them into corresponding objects
                        int cpos = dxl_read_word(id, s->gaddr(REG_CURRENT_POSITION), ack);
                        s->updateValue(REG_CURRENT_POSITION, cpos);
                        s->setError(dxl_get_rxpacket_error());
                        updateErrorCount(dxl_get_com_error());
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
                                    double angle_abs = abs(angle);
                                    int speed = (movingSpeed + static_cast<int>(k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        // SPEED
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                        updateErrorCount(dxl_get_com_error());
                                        dxl_print_error();
                                        s->setError(dxl_get_rxpacket_error());

                                        // POS
                                        if (angle >= 0)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getSteps() - 1, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), 0, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }

                                        //std::cout << "pos:" << cpos << " Movingspeed: " << speed << "  CurrentSpeed: " << s->getCurrentSpeed() << " |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
                                    }
                                    else // STOP
                                    {
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), movingSpeed, ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error());
                                        dxl_print_error();

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition(), ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error());
                                        dxl_print_error();

                                        //std::cout << "[STOP] pos:" << cpos << " speed: " << speed << "   |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
                                        s->commitValue(REG_GOAL_POSITION, 0);
                                    }
                                }
                                else // if (s->getCwAngleLimit() == 0 && s->getCcwAngleLimit() == 0) // WHEEL MODE
                                {
                                    double step = 360.0 / s->getSteps();
                                    double angle = static_cast<double>(gpos - cpos) * step;

                                    if (angle > 180) angle -= 360;
                                    else if (angle < -180) angle += 360;
                                    double angle_abs = abs(angle);

                                    int speed = (movingSpeed + static_cast<int>(k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        if (angle >= 0)
                                        {
                                            // SPEED (counter clockwise)
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            // SPEED (clockwise)
                                            speed +=  1024;
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }

                                        //std::cout << "pos:" << cpos << " speed: " << speed << "   |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
                                    }
                                    else // STOP
                                    {
                                        if (dxl_read_word(id, s->gaddr(REG_GOAL_SPEED), ack) >= 1024)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), ack, 1024);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), 0, ack);
                                            s->setError(dxl_get_rxpacket_error());
                                            updateErrorCount(dxl_get_com_error());
                                            dxl_print_error();
                                        }

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition(), ack);
                                        s->setError(dxl_get_rxpacket_error());
                                        updateErrorCount(dxl_get_com_error());
                                        dxl_print_error();

                                        //std::cout << "[STOP] pos:" << cpos << " speed: " << speed << "   |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
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
            std::cerr << "Sync loop duration: " << (loopd / 1000.0) << "ms of the " << syncloopDuration << "ms budget." << std::endl;
        else
            std::cout << "Sync loop duration: " << (loopd / 1000.0) << "ms of the " << syncloopDuration << "ms budget." << std::endl;
#endif

        if (waitd > 0.0)
        {
            std::chrono::microseconds waittime(static_cast<int>(waitd));
            std::this_thread::sleep_for(waittime);
        }
    }

    std::cout << ">> THREAD (tid: " << std::this_thread::get_id() << "') termination by 'loop exit'" << std::endl;
}
