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
 * \author Emeric Grange <emeric.grange@inria.fr>
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

    startThread();
}

DynamixelController::~DynamixelController()
{
    stopThread();
    serialTerminate();
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

int DynamixelController::serialInitialize_wrapper(std::string &deviceName, const int baud, const int serialDevice)
{
    return serialInitialize(deviceName, baud, serialDevice);
}

void DynamixelController::serialTerminate_wrapper()
{
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
    setLatency(latency);
}

void DynamixelController::autodetect_internal(int start, int stop)
{
    controllerState = state_scanning;

    // Prepare to scan, clean servoLists
    unregisterServos_internal();

    // Check start/stop boundaries
    if (start < 0 || start > (maxId - 1))
        start = 0;

    if (stop < 1 || stop > maxId || stop < start)
        stop = maxId;

    // Bring RX packet timeout down to scan way faster
    setLatency(8);

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
    setLatency(LATENCY_TIME_DEFAULT);

    controllerState = state_scanned;
}

void DynamixelController::run()
{
    std::cout << "DynamixelController::run(port: '" << serialGetCurrentDevice() << "' | tid: '" << std::this_thread::get_id() << "')" << std::endl;
    std::chrono::time_point<std::chrono::system_clock> start, end;

    while (syncloopRunning)
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
                registerServo_internal((Servo *)(m.p));
                break;
            case ctrl_device_unregister:
                unregisterServo_internal((Servo *)(m.p));
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
/*
            case ctrl_pause:
            case ctrl_resume:
                pauseThread_internal();
                break;
            case ctrl_stop:
                stopThread_internal();
                break;
*/
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
            int actionProgrammed, rebootProgrammed, refreshProgrammed, resetProgrammed;
            s->getActions(actionProgrammed, rebootProgrammed, refreshProgrammed, resetProgrammed);

            if (refreshProgrammed == 1)
            {
                // Every servo register value will be updated
                updateList.push_back(s->getId());
                std::cout << "Refresh servo #" << s->getId() << " registers"<< std::endl;
            }

            if (actionProgrammed == 1)
            {
                dxl_action(s->getId());
                std::cout << "Action for servo #" << s->getId() << std::endl;
            }

            if (rebootProgrammed == 1)
            {
                // Remove servo from sync/update lists; Need to be added again after reboot!
                for (std::vector <int>::iterator it = updateList.begin(); it != updateList.end();)
                {
                    if (*it == s->getId())
                    { updateList.erase(it); }
                    else
                    { it++; }
                }
                for (std::vector <int>::iterator it = syncList.begin(); it != syncList.end();)
                {
                    if (*it == s->getId())
                    { syncList.erase(it); }
                    else
                    { it++; }
                }

                // Reboot
                dxl_reboot(s->getId());
                std::cout << "Rebooting servo #" << s->getId() << "..." << std::endl;

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), NULL, s->getId(), 0};
                sendMessage(&m);
            }

            if (resetProgrammed > 0)
            {
                // Remove servo from sync/update lists; Need to be added again after reset!
                for (std::vector <int>::iterator it = updateList.begin(); it != updateList.end();)
                {
                    if (*it == s->getId())
                    { updateList.erase(it); }
                    else
                    { it++; }
                }
                for (std::vector <int>::iterator it = syncList.begin(); it != syncList.end();)
                {
                    if (*it == s->getId())
                    { syncList.erase(it); }
                    else
                    { it++; }
                }

                // Reset
                dxl_reset(s->getId(), resetProgrammed);
                std::cout << "Resetting servo #" << s->getId() << " (setting: " << resetProgrammed << ")..." << std::endl;

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), NULL, s->getId(), 1};
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
                controllerState = state_reading;
                int id = (*itr);

                for (auto s: servoList)
                {
                    if (s->getId() == id)
                    {
                        for (int ctid = 1; ctid < s->getRegisterCount(); ctid++)
                        {
                            int regname = getRegisterName(s->getControlTable(), ctid);
                            int regaddr = getRegisterAddr(s->getControlTable(), regname);
                            int regsize = getRegisterSize(s->getControlTable(), regname);

                            //std::cout << "Reading value for reg [" << ctid << "] name: '" << getRegisterNameTxt(regname)
                            //          << "' addr: '" << regaddr << "' size: '" << regsize << "'" << std::endl;

                            if (regsize == 1)
                            {
                                s->updateValue(regname, dxl_read_byte(id, regaddr));
                            }
                            else //if (regsize == 2)
                            {
                                s->updateValue(regname, dxl_read_word(id, regaddr));
                            }
                            s->setError(dxl_get_rxpacket_error());
                            errors += dxl_get_com_error();
                            dxl_print_error();
                        }

                        // Once all registers are read, remove the servo from the "updateList"
                        itr = updateList.erase(itr);
                    }
                }
            }
            controllerState = state_ready;
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

                ServoDynamixel *s = (ServoDynamixel*)s_raw;

                if (s->getId() == id)
                {
                    for (int ctid = 0; ctid < s->getRegisterCount(); ctid++)
                    {
                        int regname = getRegisterName(s->getControlTable(), ctid);

                        if ((s->getSpeedMode() == SPEED_AUTO && (regname != REG_GOAL_POSITION || regname != REG_GOAL_SPEED)) == false)
                        {
                            if (s->getValueCommit(regname) == 1)
                            {
                                int regaddr = getRegisterAddr(s->getControlTable(), regname);
                                int regsize = getRegisterSize(s->getControlTable(), regname);

                                //std::cout << "Writing value '" << s->getValue(regname) << "' for reg [" << ctid << "] name: '" << getRegisterNameTxt(regname)
                                //          << "' addr: '" << regaddr << "' size: '" << regsize << "'" << std::endl;

                                if (regsize == 1)
                                {
                                    dxl_write_byte(id, regaddr, s->getValue(regname));
                                }
                                else //if (regsize == 2)
                                {
                                    dxl_write_word(id, regaddr, s->getValue(regname));
                                }

                                s->commitValue(regname, 0);
                                s->setError(dxl_get_rxpacket_error());
                                errors += dxl_get_com_error();
                                dxl_print_error();
                            }
                        }
                    }

                    // 1Hz "low priority" loop
                    if ((syncloopCounter - cumulid) == 0)
                    {
                        // Read voltage
                        s->updateValue(REG_CURRENT_VOLTAGE, dxl_read_byte(id, s->gaddr(REG_CURRENT_VOLTAGE)));
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
                        dxl_print_error();

                        // Read temp
                        s->updateValue(REG_CURRENT_TEMPERATURE, dxl_read_byte(id, s->gaddr(REG_CURRENT_TEMPERATURE)));
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
                        dxl_print_error();
                    }

                    // x/4 Hz "feedback" loop
                    if ((syncloopCounter - cumulid) % 4 == 0)
                    {
                        s->updateValue(REG_CURRENT_SPEED, dxl_read_word(id, s->gaddr(REG_CURRENT_SPEED)));
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
                        dxl_print_error();

                        s->updateValue(REG_CURRENT_LOAD, dxl_read_word(id, s->gaddr(REG_CURRENT_LOAD)));
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
                        dxl_print_error();

                        // Read moving
                        s->updateValue(REG_MOVING, dxl_read_byte(id, s->gaddr(REG_MOVING)));
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
                        dxl_print_error();
                    }

                    // x Hz "full speed" loop
                    {
                        // Get "current" values from servo, and write them into obj
                        int cpos = dxl_read_word(id, s->gaddr(REG_CURRENT_POSITION));
                        s->updateValue(REG_CURRENT_POSITION, cpos);
                        s->setError(dxl_get_rxpacket_error());
                        errors += dxl_get_com_error();
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
                                    int speed = (movingSpeed + (k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        // SPEED
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed);
                                        errors += dxl_get_com_error();
                                        dxl_print_error();
                                        s->setError(dxl_get_rxpacket_error());

                                        // POS
                                        if (angle >= 0)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getSteps() - 1);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), 0);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }

                                        //std::cout << "pos:" << cpos << " Movingspeed: " << speed << "  CurrentSpeed: " << s->getCurrentSpeed() << " |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
                                    }
                                    else // STOP
                                    {
                                        dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), movingSpeed);
                                        s->setError(dxl_get_rxpacket_error());
                                        errors += dxl_get_com_error();
                                        dxl_print_error();

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition());
                                        s->setError(dxl_get_rxpacket_error());
                                        errors += dxl_get_com_error();
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

                                    int speed = (movingSpeed + (k * angle_abs));

                                    if (angle_abs > mot)
                                    {
                                        if (angle >= 0)
                                        {
                                            // SPEED (counter clockwise)
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            // SPEED (clockwise)
                                            speed +=  1024;
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), speed);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }

                                        //std::cout << "pos:" << cpos << " speed: " << speed << "   |   (> " << gpos << ") (angle: " << angle << ")" << std::endl;
                                    }
                                    else // STOP
                                    {
                                        if (dxl_read_word(id, s->gaddr(REG_GOAL_SPEED)) >= 1024)
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), 1024);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }
                                        else
                                        {
                                            dxl_write_word(id, s->gaddr(REG_GOAL_SPEED), 0);
                                            s->setError(dxl_get_rxpacket_error());
                                            errors += dxl_get_com_error();
                                            dxl_print_error();
                                        }

                                        dxl_write_word(id, s->gaddr(REG_GOAL_POSITION), s->getGoalPosition());
                                        s->setError(dxl_get_rxpacket_error());
                                        errors += dxl_get_com_error();
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
        std::cout << "Sync loop duration: " << (loopd / 1000.0) << "ms of the " << syncloopDuration << "ms budget." << std::endl;
#endif

        if (waitd > 0.0)
        {
            std::chrono::microseconds waittime(static_cast<int>(waitd));
            std::this_thread::sleep_for(waittime);
        }
    }
}
