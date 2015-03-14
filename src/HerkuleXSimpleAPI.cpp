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
 * \file HerkuleXSimpleAPI.cpp
 * \date 19/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "HerkuleXSimpleAPI.h"
#include "ControlTables.h"
#include "ControlTablesDynamixel.h"
#include "ControlTablesHerkuleX.h"

// C++ standard libraries
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>

HerkuleXSimpleAPI::HerkuleXSimpleAPI(int servos)
{
    if (servos != SERVO_UNKNOWN)
    {
        std::cout << std::endl;

        if (servos >= SERVO_HERKULEX)
        {
            ackPolicy = 1;
            maxId = 253;

            protocolVersion = 1;
            servoSerie = SERVO_DRS;

            if (servos == SERVO_DRS_0402 || servos == SERVO_DRS_0602)
            {
                ct = DRS0x02_control_table;
            }
            else if (servos == SERVO_DRS_0401 || servos == SERVO_DRS_0601)
            {
                ct = DRS0x01_control_table;
            }
            else
            {
                ct = DRS0101_control_table;
            }

            std::cout << "- Using HerkuleX communication protocol" << std::endl;
        }
        else //if (servos >= SERVO_DYNAMIXEL)
        {
            ackPolicy = 2;
            maxId = 252;

            if (servos >= SERVO_PRO)
            {
                protocolVersion = 2;
                servoSerie = SERVO_PRO;
                ct = PRO_control_table;
            }
            else if (servos >= SERVO_XL)
            {
                protocolVersion = 2;
                servoSerie = SERVO_XL;
                ct = XL320_control_table;
            }
            else // if (servos >= SERVO_MX)
            {
                // We set the servo serie to 'MX' which is the more capable of the Dynamixel v1 serie
                protocolVersion = 1;
                servoSerie = SERVO_MX;
                ct = MX_control_table;

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

HerkuleXSimpleAPI::~HerkuleXSimpleAPI()
{
    disconnect();
}

int HerkuleXSimpleAPI::connect(std::string &devicePath, const int baud, const int serialDevice)
{
    this->serialDevice = serialDevice;
    return serialInitialize(devicePath, baud);
}

void HerkuleXSimpleAPI::disconnect()
{
    serialTerminate();
}

bool HerkuleXSimpleAPI::checkId(const int id, const bool broadcast)
{
    bool status = false;

    if ((id >= 0 && id <= maxId) ||
        (id == BROADCAST_ID && broadcast == true))
    {
        status = true;
    }
    else
    {
        if (id == BROADCAST_ID && broadcast == false)
        {
            std::cerr << "Error: Broadcast ID is disabled for the current instruction." << std::endl;
        }
        else
        {
            std::cerr << "Error: ID '" << id << "' is out of [0;" << maxId << "] boundaries." << std::endl;
        }
    }

    return status;
}

std::vector <int> HerkuleXSimpleAPI::servoScan(int start, int stop)
{
    // Check start/stop boundaries
    if (start < 0 || start > (maxId - 1))
        start = 0;

    if (stop < 1 || stop > maxId || stop < start)
        stop = maxId;

    std::cout << std::endl << "> Scanning for HerkuleX devices on '" << serialGetCurrentDevice() << "'... Range is [" << start << "," << stop << "]" << std::endl;

    // A vector of HerkuleX IDs found during the scan
    std::vector <int> ids;

    for (int id = start; id <= stop; id++)
    {
        PingResponse pingstats;

        // If the ping gets a response, then we have found a servo
        if (hkx_ping(id, &pingstats) == true)
        {
            setLed(id, 1, LED_GREEN);

            ids.push_back(id);

            std::cout << std::endl;
            std::cout << "[#" << id << "] HerkuleX servo found!" << std::endl;
            std::cout << "[#" << id << "] model: " << pingstats.model_number << " (" << hkx_get_model_name(pingstats.model_number) << ")" << std::endl;

            // Other informations, not printed by default:
            std::cout << "[#" << id << "] firmware: " << pingstats.firmware_version << std::endl;
            //std::cout << "[#" << id << "] position: " << hkx_read_current_position(id) << std::endl;
            //std::cout << "[#" << id << "] speed: " << hkx_read_current_speed(id) << std::endl;
            //std::cout << "[#" << id << "] torque: " << hkx_read_current_torque(id) << std::endl;
            //std::cout << "[#" << id << "] load: " << hkx_read_current_load(id) << std::endl;
            //std::cout << "[#" << id << "] baudrate: " << hkx_get_baud(id) << std::endl;

            setLed(id, 0);
        }
        else
        {
            std::cout << ".";
        }
    }

    std::cout << std::endl;
    return ids;
}

bool HerkuleXSimpleAPI::ping(const int id, PingResponse *status)
{
    return hkx_ping(id, status);
}

void HerkuleXSimpleAPI::reboot(const int id)
{
    if (checkId(id) == true)
    {
        hkx_reboot(id);
    }
}

void HerkuleXSimpleAPI::reset(const int id, const int setting)
{
    if (checkId(id) == true)
    {
        hkx_reset(id, setting);
    }
}

int HerkuleXSimpleAPI::readModelNumber(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_MODEL_NUMBER);

        value = hkx_read_word(id, addr, REGISTER_ROM);
        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::readFirmwareVersion(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_FIRMWARE_VERSION);

        value = hkx_read_word(id, addr, REGISTER_ROM);
        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::changeId(const int old_id, const int new_id)
{
    int status = 0;

    // Check 'old' ID
    if (checkId(old_id, false) == true)
    {
        // Check 'new' ID // Valid IDs are in range [0:maxId]
        if ((new_id >= 0) && (new_id <= maxId))
        {
            // If the ping get a response, then we already have a servo on the new id
            hkx_ping(new_id);

            if (hkx_get_com_status() == COMM_RXSUCCESS)
            {
                std::cout << "Cannot set new ID '" << new_id << "' for this servo: already in use" << std::endl;
            }
            else
            {
                int addr_rom = getRegisterAddr(ct, REG_ID, REGISTER_ROM);
                hkx_write_byte(old_id, addr_rom, new_id, REGISTER_ROM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }

                int addr_ram = getRegisterAddr(ct, REG_ID, REGISTER_RAM);
                hkx_write_byte(old_id, addr_ram, new_id, REGISTER_RAM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }
            }
        }
        else
        {
            std::cerr << "Cannot set new ID '" << new_id << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int HerkuleXSimpleAPI::changeBaudRate(const int id, const int baudnum)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid baudnums are in range [0:34]
        if ((baudnum >= 0) && (baudnum <= 34))
        {
            int addr_rom = getRegisterAddr(ct, REG_BAUD_RATE, REGISTER_ROM);
            hkx_write_byte(id, addr_rom, baudnum, REGISTER_ROM);
            if (hkx_print_error() == 0)
            {
                status = 1;
            }

            int addr_ram = getRegisterAddr(ct, REG_BAUD_RATE, REGISTER_RAM);
            hkx_write_byte(id, addr_ram, baudnum, REGISTER_RAM);
            if (hkx_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set new baudnum '" << baudnum << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

void HerkuleXSimpleAPI::getMinMaxPositions(const int id, int &min, int &max)
{
    if (checkId(id, false) == true)
    {
        int addr_min_ram = getRegisterAddr(ct, REG_MIN_POSITION, REGISTER_RAM);
        int addr_max_ram = getRegisterAddr(ct, REG_MAX_POSITION, REGISTER_RAM);

        min = hkx_read_word(id, addr_min_ram, REGISTER_RAM);
        if (hkx_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((min < 0) || (min > 4095))
            {
                min = -1;
            }
        }
        else
        {
            min = -1;
        }

        max = hkx_read_word(id, addr_max_ram, REGISTER_RAM);
        if (hkx_print_error() == 0)
        {
            // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
            if ((max < 0) || (max > 4095))
            {
                max = -1;
            }
        }
        else
        {
            max = -1;
        }
    }
}

int HerkuleXSimpleAPI::setMinMaxPositions(const int id, const int min, const int max)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
        if ((min < 0) || (min > 4095) || (max < 0) || (max > 4095))
        {
            std::cerr << "Cannot set new min/max positions: '" << min << "/" << max << "' for this servo: out of range" << std::endl;
        }
        else
        {
            // ROM
            {
                int addr_min_rom = getRegisterAddr(ct, REG_MIN_POSITION, REGISTER_ROM);
                int addr_max_rom = getRegisterAddr(ct, REG_MAX_POSITION, REGISTER_ROM);

                // Write min value
                hkx_write_word(id, addr_min_rom, min, REGISTER_ROM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }

                // Write max value
                hkx_write_word(id, addr_max_rom, max, REGISTER_ROM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }
            }

            // RAM
            {
                int addr_min_ram = getRegisterAddr(ct, REG_MIN_POSITION, REGISTER_RAM);
                int addr_max_ram = getRegisterAddr(ct, REG_MAX_POSITION, REGISTER_RAM);

                // Write min value
                hkx_write_word(id, addr_min_ram, min, REGISTER_RAM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }

                // Write max value
                hkx_write_word(id, addr_max_ram, max, REGISTER_RAM);
                if (hkx_print_error() == 0)
                {
                    status = 1;
                }
            }
        }
    }

    return status;
}

int HerkuleXSimpleAPI::getTorqueEnabled(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr_ram = getRegisterAddr(ct, REG_TORQUE_ENABLE, REGISTER_RAM);
        value = hkx_read_byte(id, addr_ram, REGISTER_RAM);

        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::setTorqueEnabled(const int id, int torque)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Normalize value, 1 means 'torque on' for Dynamixel devices
        if (torque == 1)
        {
            torque = 0x60;
        }

        // Valid torque are in range [0:254]
        if (torque == 0x00 || torque == 0x40 || torque == 0x60)
        {
            int addr_ram = getRegisterAddr(ct, REG_TORQUE_ENABLE, REGISTER_RAM);
            hkx_write_byte(id, addr_ram, torque, REGISTER_RAM);
            if (hkx_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set torque_enabled: '" << torque << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int HerkuleXSimpleAPI::getLed(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_LED, REGISTER_RAM);
        value = hkx_read_byte(id, addr, REGISTER_RAM);

        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::setLed(const int id, int led, const int color)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Normalize value
        if (led >= 1)
        {
            led = 0;

            if (color & LED_GREEN)
            { led += 0x01; }
            if (color & LED_BLUE)
            { led += 0x02; }
            if (color & LED_RED)
            { led += 0x04; }
        }
        else
        {
            led = 0;
        }

        // Execute command
        int addr = getRegisterAddr(ct, REG_LED, REGISTER_RAM);
        if (addr >= 0)
        {
            hkx_write_byte(id, addr, led, REGISTER_RAM);

            // Check for error
            if (hkx_print_error() == 0)
            {
                status = 1;
            }
        }
    }

    return status;
}

int HerkuleXSimpleAPI::turn(const int id, const int velocity)
{
    int status = 0;

    if (checkId(id) == true)
    {
        hkx_i_jog(id, 1, velocity);
        if (hkx_print_error() == 0)
        {
            status = 1;
        }
    }

    return status;
}

int HerkuleXSimpleAPI::getGoalPosition(const int id)
{
    int value = -1;

    if (checkId(id) == true)
    {
        int addr = getRegisterAddr(ct, REG_GOAL_POSITION, REGISTER_RAM);

        value = hkx_read_byte(id, addr, REGISTER_RAM);
        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::setGoalPosition(const int id, const int position)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
        if ((position >= 0) && (position <= 4095))
        {
            hkx_i_jog(id, 0, position);
            if (hkx_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set goal position: '" << position << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int HerkuleXSimpleAPI::setGoalPosition(const int id, const int position, const int speed)
{
    int status = 0;

    //TODO Set goal speed, and if success proceed to set goal position

    if (checkId(id) == true)
    {
        // Valid positions are in range [0:1023] for most servo series, and [0:4095] for high-end servo series
        if ((position >= 0) && (position <= 4095))
        {
            hkx_i_jog(id, 0, position);
            if (hkx_print_error() == 0)
            {
                status = 1;
            }
        }
        else
        {
            std::cerr << "Cannot set goal position: '" << position << "' for this servo: out of range" << std::endl;
        }
    }

    return status;
}

int HerkuleXSimpleAPI::readCurrentPosition(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_TEMPERATURE, REGISTER_RAM);

        value = hkx_read_byte(id, addr, REGISTER_RAM);
        if (hkx_print_error() != 0)
        {
            value = -1;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::readCurrentSpeed(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        //
    }

    return value;
}

int HerkuleXSimpleAPI::readCurrentLoad(const int id)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        //
    }

    return value;
}

double HerkuleXSimpleAPI::readCurrentTemperature(const int id)
{
    double value = 0.0;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_TEMPERATURE, REGISTER_RAM);

        value = static_cast<double>(hkx_read_byte(id, addr, REGISTER_RAM));
        if (hkx_print_error() == 0)
        {
            // FIXME temperature is using a non linear scale
            value *= 0.326;
        }
        else
        {
            value = 0.0;
        }
    }

    return value;
}

double HerkuleXSimpleAPI::readCurrentVoltage(const int id)
{
    double value = 0.0;

    if (checkId(id, false) == true)
    {
        int addr = getRegisterAddr(ct, REG_CURRENT_VOLTAGE, REGISTER_RAM);

        value = static_cast<double>(hkx_read_byte(id, addr, REGISTER_RAM));
        if (hkx_print_error() == 0)
        {
            value *= 0.074074;
        }
        else
        {
            value = 0.0;
        }
    }

    return value;
}

int HerkuleXSimpleAPI::getSetting(const int id, const int reg_name, int reg_type, const int device)
{
    int value = -1;

    if (checkId(id, false) == true)
    {
        // Device detection
        const int (*cctt)[8] = getRegisterTable(device);

        if (cctt == NULL)
        {
            // Using default control table from this SimpleAPI isntance
            cctt = ct;
        }

        if (cctt)
        {
            // Find register's informations (addr, size...)
            RegisterInfos infos;
            if (getRegisterInfos(cctt, reg_name, infos) == 1)
            {
                // Register type
                if (reg_type == REGISTER_AUTO)
                {
                    if (infos.reg_addr_rom >= 0 && infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_RAM; // Cannot return both values, so default is set to RAM
                    else if (infos.reg_addr_rom >= 0)
                        reg_type = REGISTER_ROM;
                    else if (infos.reg_addr_ram >= 0)
                        reg_type = REGISTER_RAM;
                }

                // Read value
                if (reg_type == REGISTER_ROM)
                {
                    if (infos.reg_size == 1)
                    {
                        value = hkx_read_byte(id, infos.reg_addr_rom, REGISTER_ROM);
                    }
                    else if (infos.reg_size == 2)
                    {
                        value = hkx_read_word(id, infos.reg_addr_rom, REGISTER_ROM);
                    }
                }
                else if (reg_type == REGISTER_RAM)
                {
                    if (infos.reg_size == 1)
                    {
                        value = hkx_read_byte(id, infos.reg_addr_ram, REGISTER_RAM);
                    }
                    else if (infos.reg_size == 2)
                    {
                        value = hkx_read_word(id, infos.reg_addr_ram, REGISTER_RAM);
                    }
                }

                // Check status
                if (hkx_print_error() == 0)
                {
                    // Check value
                    if (value < infos.reg_value_min && value > infos.reg_value_max)
                    {
                        value = -1;
                    }
                }
                else
                {
                    value = -1;
                }
            }
            else
            {
                std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [CONTROL TABLE ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << id << "] getSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [DEVICE ID ERROR]" << std::endl;
    }

    return value;
}

int HerkuleXSimpleAPI::setSetting(const int id, const int reg_name, const int reg_value, int reg_type, const int device)
{
    int status = 0;

    if (checkId(id) == true)
    {
        // Device detection
        const int (*cctt)[8] = getRegisterTable(device);

        if (cctt == NULL)
        {
            // Using default control table from this SimpleAPI isntance
            cctt = ct;
        }

        if (cctt)
        {
            // Find register's informations (addr, size...)
            RegisterInfos infos;
            if (getRegisterInfos(cctt, reg_name, infos) == 1)
            {
                // Check if we have permission to write into this register
                if (infos.reg_access_mode == READ_WRITE)
                {
                    // Check value
                    if (reg_value >= infos.reg_value_min && reg_value <= infos.reg_value_max)
                    {
                        // Register type
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
                            // Write value
                            if (infos.reg_size == 1)
                            {
                                hkx_write_byte(id, infos.reg_addr_rom, reg_value, REGISTER_ROM);
                            }
                            else if (infos.reg_size == 2)
                            {
                                hkx_write_word(id, infos.reg_addr_rom, reg_value, REGISTER_ROM);
                            }

                            // Check for error
                            if (hkx_print_error() == 0)
                            {
                                status = 1;
                            }
                        }

                        if (reg_type == REGISTER_RAM || reg_type == REGISTER_BOTH)
                        {
                            // Write value
                            if (infos.reg_size == 1)
                            {
                                hkx_write_byte(id, infos.reg_addr_ram, reg_value, REGISTER_RAM);
                            }
                            else if (infos.reg_size == 2)
                            {
                                hkx_write_word(id, infos.reg_addr_ram, reg_value, REGISTER_RAM);
                            }

                            // Check for error
                            if (hkx_print_error() == 0)
                            {
                                status = 1;
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << " to '" << reg_value
                                  << "')  [VALUE ERROR]! min/max(" << infos.reg_value_min << "/" << infos.reg_value_max << ")" << std::endl;
                    }
                }
                else
                {
                    std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER ACCESS ERROR]" << std::endl;
                }
            }
            else
            {
                std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [REGISTER NAME ERROR]" << std::endl;
            }
        }
        else
        {
            std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [CONTROL TABLE ERROR]" << std::endl;
        }
    }
    else
    {
        std::cerr << "[#" << id << "] setSetting(reg " << reg_name << "/" << getRegisterNameTxt(reg_name) << ") [DEVICE ID ERROR]" << std::endl;
    }

    return status;
}
