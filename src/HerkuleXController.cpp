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
 * \file HerkuleXController.cpp
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "HerkuleXController.h"
#include "minitraces.h"

// C++ standard libraries
#include <chrono>
#include <thread>
#include <mutex>

// Enable latency timer
//#define LATENCY_TIMER

HerkuleXController::HerkuleXController(int ctrlFrequency, int servoSerie):
    ControllerAPI(ctrlFrequency)
{
    this->servoSerie = servoSerie;
}

HerkuleXController::~HerkuleXController()
{
    disconnect();
}

void HerkuleXController::updateInternalSettings()
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

int HerkuleXController::connect(std::string &devicePath, const int baud, const int serialDevice)
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

void HerkuleXController::disconnect()
{
    stopThread();
    serialTerminate();
}

std::string HerkuleXController::serialGetCurrentDevice_wrapper()
{
    return serialGetCurrentDevice();
}

std::vector <std::string> HerkuleXController::serialGetAvailableDevices_wrapper()
{
    return serialGetAvailableDevices();
}

void HerkuleXController::serialSetLatency_wrapper(int latency)
{
    serialSetLatency(latency);
}

void HerkuleXController::autodetect_internal(int start, int stop)
{
    setState(state_scanning);

    // Prepare to scan
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

    TRACE_INFO(CAPI, "HKX ctrl_device_autodetect(port: '%s' / tid: '%i')",
               serialGetCurrentDevice().c_str(), std::this_thread::get_id());

    TRACE_INFO(CAPI, "> THREADED Scanning for HKX devices on '%s', range is [%i,%i[",
               serialGetCurrentDevice().c_str(), start, stop);

    for (int id = start; id <= stop; id++)
    {
        PingResponse pingstats;

        // If the ping gets a response, then we have found a servo
        if (hkx_ping(id, &pingstats) == true)
        {
            //setLed(id, 1, LED_GREEN);

            int serie, model;
            hkx_get_model_infos(pingstats.model_number, serie, model);
            ServoHerkuleX *servo = nullptr;

            TRACE_INFO(HKX, "[#%i] %s servo found!", id, hkx_get_model_name(pingstats.model_number).c_str());

            // Instanciate the device found
            switch (serie)
            {
            case SERVO_DRS:
                servo = new ServoDRS(id, pingstats.model_number);
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

    setState(state_scanned);
}

void HerkuleXController::run()
{
    TRACE_INFO(CAPI, "HerkuleXController::run(port: '%s' / tid: '%i')",
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
                TRACE_WARNING(HKX, "Unknown message type: '%i'", m.msg);
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
                TRACE_INFO(HKX, "Refresh servo #%i registers", id);
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
                hkx_reboot(id, ack);
                TRACE_INFO(HKX, "Rebooting servo #%i...", id);

                miniMessages m {ctrl_device_delayed_add, std::chrono::system_clock::now() + std::chrono::seconds(2), nullptr, id, 1};
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
                hkx_reset(id, resetProgrammed, ack);
                TRACE_INFO(HKX, "Resetting servo #%i (setting: %i)...", id, resetProgrammed);

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
                            struct RegisterInfos reg;
                            int reg_name = getRegisterName(s->getControlTable(), ctid);
                            getRegisterInfos(s->getControlTable(), reg_name, reg);

                            TRACE_1(HKX, "Reading value for reg [%i] name: '%s' addr: '%i' size: '%i'", ctid, getRegisterNameTxt(reg_name).c_str(), reg.reg_addr, reg.reg_size);

                            int reg_type = REGISTER_AUTO;
                            if (reg.reg_addr_rom >= 0 && reg.reg_addr_ram >= 0)
                                reg_type = REGISTER_BOTH;
                            else if (reg.reg_addr_rom >= 0)
                                reg_type = REGISTER_ROM;
                            else if (reg.reg_addr_ram >= 0)
                                reg_type = REGISTER_RAM;

                            if (reg.reg_size == 1)
                            {
                                if (reg_type == REGISTER_BOTH)
                                {
                                    s->updateValue(reg_name, hkx_read_byte(id, reg.reg_addr_rom, REGISTER_ROM, ack), REGISTER_ROM);
                                    s->updateValue(reg_name, hkx_read_byte(id, reg.reg_addr_ram, REGISTER_RAM, ack), REGISTER_RAM);
                                }
                                else if (reg_type == REGISTER_ROM)
                                {
                                    s->updateValue(reg_name, hkx_read_byte(id, reg.reg_addr_rom, REGISTER_ROM, ack), REGISTER_ROM);
                                }
                                else if (reg_type == REGISTER_RAM)
                                {
                                    s->updateValue(reg_name, hkx_read_byte(id, reg.reg_addr_ram, REGISTER_RAM, ack), REGISTER_RAM);
                                }
                            }
                            else //if (reg.reg_size == 2)
                            {
                                if (reg_type == REGISTER_BOTH)
                                {
                                    s->updateValue(reg_name, hkx_read_word(id, reg.reg_addr_rom, REGISTER_ROM, ack), REGISTER_ROM);
                                    s->updateValue(reg_name, hkx_read_word(id, reg.reg_addr_ram, REGISTER_RAM, ack), REGISTER_RAM);
                                }
                                else if (reg_type == REGISTER_ROM)
                                {
                                    s->updateValue(reg_name, hkx_read_word(id, reg.reg_addr_rom, REGISTER_ROM, ack), REGISTER_ROM);
                                }
                                else if (reg_type == REGISTER_RAM)
                                {
                                    s->updateValue(reg_name, hkx_read_word(id, reg.reg_addr_ram, REGISTER_RAM, ack), REGISTER_RAM);
                                }
                            }

                            s->setError(hkx_get_rxpacket_error());
                            s->setStatus(hkx_get_rxpacket_status_detail());
                            updateErrorCount(hkx_get_com_error_count());
                            hkx_print_error();
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

                ServoHerkuleX *s = static_cast<ServoHerkuleX*>(s_raw);

                if (s->getId() == id)
                {
                    int ack = s->getStatusReturnLevel();

                    // Unregister device if it reach an error count too high
                    // Count must be high enough to avoid "false positive": device producing a lot of errors but still present on the serial link
                    if (s->getErrorCount() > 16)
                    {
                        TRACE_ERROR(HKX, "Device #%i has an error count too high and is going to be unregistered from its controller on '%s'...", id, serialGetCurrentDevice().c_str());
                        unregisterServo(s);
                        continue;
                    }

                    // Commit register modifications
                    for (int ctid = 0; ctid < s->getRegisterCount(); ctid++)
                    {
                        int regname = getRegisterName(s->getControlTable(), ctid);
                        int regsize = getRegisterSize(s->getControlTable(), regname);

                        if (s->getValueCommit(regname, REGISTER_ROM) == 1)
                        {
                            int regaddr = getRegisterAddr(s->getControlTable(), regname, REGISTER_ROM);

                            TRACE_1(HKX, "Writing ROM value '%i' for reg [%i] name: '%s' addr: '%i' size: '%i'",
                                    s->getValue(regname, REGISTER_ROM), ctid, getRegisterNameTxt(regname).c_str(), regaddr, regsize);

                            if (regsize == 1)
                            {
                                hkx_write_byte(id, regaddr, s->getValue(regname, REGISTER_ROM), REGISTER_ROM, ack);
                            }
                            else //if (regsize == 2)
                            {
                                hkx_write_word(id, regaddr, s->getValue(regname, REGISTER_ROM), REGISTER_ROM, ack);
                            }

                            s->setError(hkx_get_rxpacket_error());
                            s->setStatus(hkx_get_rxpacket_status_detail());
                            s->commitValue(regname, 0, REGISTER_ROM);
                            updateErrorCount(hkx_get_com_error_count());
                            hkx_print_error();

                            if (regname == REG_ID)
                            {
                                if (s->changeInternalId(s->getValue(regname)) == 1)
                                {
                                    s->reboot();
                                }
                            }
                        }

                        if (s->getValueCommit(regname, REGISTER_RAM) == 1)
                        {
                            int regaddr = getRegisterAddr(s->getControlTable(), regname, REGISTER_RAM);

                            TRACE_1(HKX, "Writing RAM value '%i' for reg [%i] name: '%s' addr: '%i' size: '%i'",
                                    s->getValue(regname, REGISTER_RAM), ctid, getRegisterNameTxt(regname).c_str(), regaddr, regsize);

                            if (regsize == 1)
                            {
                                hkx_write_byte(id, regaddr, s->getValue(regname, REGISTER_RAM), REGISTER_RAM, ack);
                            }
                            else //if (regsize == 2)
                            {
                                hkx_write_word(id, regaddr, s->getValue(regname, REGISTER_RAM), REGISTER_RAM, ack);
                            }

                            s->setError(hkx_get_rxpacket_error());
                            s->setStatus(hkx_get_rxpacket_status_detail());
                            s->commitValue(regname, 0, REGISTER_RAM);
                            updateErrorCount(hkx_get_com_error_count());
                            hkx_print_error();

                            // FIXME: probably doesn't work...
                            if (regname == REG_ID)
                            {
                                unregisterServo(s);
                                if (s->changeInternalId(s->getValue(regname)) == 1)
                                {
                                    registerServo(s);
                                }
                            }
                        }
                    }

                    // 1 Hz "low priority" update loop
                    if (((syncloopCounter - cumulid) == 0) &&
                        (ack != ACK_NO_REPLY))
                    {
                        // Read voltage
                        s->updateValue(REG_CURRENT_VOLTAGE, hkx_read_byte(id, s->gaddr(REG_CURRENT_VOLTAGE), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();

                        // Read temp
                        s->updateValue(REG_CURRENT_TEMPERATURE, hkx_read_byte(id, s->gaddr(REG_CURRENT_TEMPERATURE), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();
                    }

                    // x/4 Hz "feedback" update loop
                    if (((syncloopCounter - cumulid) % 4 == 0) &&
                        (ack != ACK_NO_REPLY))
                    {
                        s->updateValue(REG_STATUS_ERROR, hkx_read_byte(id, s->gaddr(REG_STATUS_ERROR), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();

                        s->updateValue(REG_STATUS_DETAIL, hkx_read_byte(id, s->gaddr(REG_STATUS_DETAIL), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();
/*
                        s->updateCurrentSpeed(hkx_read_word(id, s->gaddr(SERVO_CURRENT_SPEED), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();

                        s->updateCurrentLoad(hkx_read_word(id, s->gaddr(SERVO_CURRENT_LOAD), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();
*/
                    }

                    // x Hz "full speed" update loop
                    {
                        // Get "current" values from devices, and write them into corresponding objects
                        int cpos = hkx_read_word(id, s->gaddr(REG_ABSOLUTE_POSITION), REGISTER_RAM, ack);
                        s->updateValue(REG_ABSOLUTE_POSITION, cpos);
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();

                        if (s->getGoalPositionCommited() == 1)
                        {
                            int gpos = s->getGoalPosition();

                            hkx_i_jog(id, 0, gpos, ack);
                            if (hkx_print_error() == 0)
                            {
                                s->commitGoalPosition();
                            }
                        }

                        s->updateValue(REG_ABSOLUTE_GOAL_POSITION, hkx_read_word(id, s->gaddr(REG_ABSOLUTE_GOAL_POSITION), REGISTER_RAM, ack));
                        s->setError(hkx_get_rxpacket_error());
                        s->setStatus(hkx_get_rxpacket_status_detail());
                        updateErrorCount(hkx_get_com_error_count());
                        hkx_print_error();
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
            TRACE_WARNING(HKX, "Sync loop duration: %fms of the %fms budget.", (loopd / 1000.0), syncloopDuration);
        }
        else
        {
            TRACE_INFO(HKX, "Sync loop duration: %fms of the %fms budget.", (loopd / 1000.0), syncloopDuration);
        }
#endif // LATENCY_TIMER

        if (waitd > 0.0)
        {
            std::chrono::microseconds waittime(static_cast<int>(waitd));
            std::this_thread::sleep_for(waittime);
        }
    }

    TRACE_INFO(HKX, ">> THREAD (tid: '%i') termination by 'loop exit'", std::this_thread::get_id());
}
