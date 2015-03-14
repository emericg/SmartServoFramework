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
 * \file ControlTables.cpp
 * \date 21/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ControlTables.h"
#include "ControlTablesDynamixel.h"
#include "ControlTablesHerkuleX.h"

// C++ standard libraries
#include <iostream>
#include <cmath>

const int (*getRegisterTable(const int servo_model))[8]
{
    const int (*ct)[8] = NULL;

    if (servo_model != SERVO_UNKNOWN)
    {
        if (servo_model >= SERVO_HERKULEX)
        {
            if (servo_model == SERVO_DRS_0101 ||
                servo_model == SERVO_DRS_0201)
            {
                ct = DRS0101_control_table;
            }
            else if (servo_model == SERVO_DRS_0401 ||
                     servo_model == SERVO_DRS_0601)
            {
                ct = DRS0x01_control_table;
            }
            else if (servo_model == SERVO_DRS_0402 ||
                     servo_model == SERVO_DRS_0602)
            {
                ct = DRS0x02_control_table;
            }
        }
        else if (servo_model >= SERVO_DYNAMIXEL)
        {
            if (servo_model < SERVO_EX)
            {
                ct = AXDXRX_control_table;
            }
            else if (servo_model < SERVO_MX)
            {
                ct = EX_control_table;
            }
            else if (servo_model < SERVO_XL)
            {
                ct = MX_control_table;
            }
            else if (servo_model < SERVO_PRO)
            {
                ct = XL320_control_table;
            }
            else if (servo_model < SENSOR_DYNAMIXEL)
            {
                ct = PRO_control_table;
            }
            else if (servo_model < 100)
            {
                if (servo_model == SENSOR_AXS1)
                {
                    ct = AXS1_control_table;
                }
                else if (servo_model == SENSOR_IR_ARRAY)
                {
                    ct = IR_ARRAY_control_table;
                }
            }
        }
    }

    if (ct == NULL)
    {
        std::cerr << "Unable to find a suitable 'Control Table' for servo_model: '" << servo_model << "'" << std::endl;
    }

    return ct;
}

const int (*getRegisterTable(const int servo_serie, const int servo_model))[8]
{
    const int (*ct)[8] = NULL;

    if (servo_serie >= SERVO_HERKULEX)
    {
        if (servo_model == SERVO_DRS_0101 ||
            servo_model == SERVO_DRS_0201)
        {
            ct = DRS0101_control_table;
        }
        else if (servo_model == SERVO_DRS_0401 ||
                 servo_model == SERVO_DRS_0601)
        {
            ct = DRS0x01_control_table;
        }
        else if (servo_model == SERVO_DRS_0402 ||
                 servo_model == SERVO_DRS_0602)
        {
            ct = DRS0x02_control_table;
        }
    }
    else if (servo_serie >= SERVO_DYNAMIXEL)
    {
        if (servo_serie == SERVO_AX ||
            servo_serie == SERVO_DX ||
            servo_serie == SERVO_RX)
        {
            ct = AXDXRX_control_table;
        }
        else if (servo_serie == SERVO_EX || servo_model == SERVO_EX106)
        {
            ct = EX_control_table;
        }
        else if (servo_serie == SERVO_MX)
        {
            ct = MX_control_table;
        }
        else if (servo_serie == SERVO_XL || servo_model == SERVO_XL320)
        {
            ct = XL320_control_table;
        }
        else if (servo_serie == SERVO_PRO)
        {
            ct = PRO_control_table;
        }
        else if (servo_serie == SENSOR_DYNAMIXEL)
        {
            if (servo_model == SENSOR_AXS1)
            {
                ct = AXS1_control_table;
            }
            else if (servo_model == SENSOR_IR_ARRAY)
            {
                ct = IR_ARRAY_control_table;
            }
        }
    }

    if (ct == NULL)
    {
        std::cerr << "Unable to find a suitable 'Control Table' for servo_serie: '" << servo_serie << "' servo_model: '" << servo_model << "'" << std::endl;
    }

    return ct;
}

unsigned getRegisterCount(const int ct[][8])
{
    unsigned count = 0;

    if (ct != NULL)
    {
        // Register count // Horrible hack
        for (unsigned i = 0; i < 64; i++)
        {
            if (ct[i][0] == 999)
            {
                //std::cout << "Control table size is: " << i << std::endl;
                count = i;
                break; // exit search loop
            }
        }
    }

    return count;
}

int getRegisterInfos(const int ct[][8], const int reg_name, RegisterInfos &infos)
{
    int status = -1;

    if (ct != NULL)
    {
        // DEBUG
        //std::cout << "Control table size is: " << getRegisterCount() << std::endl;

        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            // DEBUG
            //std::cout << "Control table [" << i << "/" << getRegisterCount() << "] value: "<< ct[i][0] << std::endl;

            if (ct[i][0] == reg_name) // match register name
            {
                infos.reg_index = i;

                int rom = ct[i][3];
                int ram = ct[i][4];

                if (rom != -1)
                    infos.reg_addr = rom;
                else if (ram != -1)
                    infos.reg_addr = ram;

                infos.reg_addr_rom = rom;
                infos.reg_addr_ram = ram;
                infos.reg_size = ct[i][1];
                infos.reg_access_mode = ct[i][2];
                infos.reg_value_def = ct[i][5];
                infos.reg_value_min = ct[i][6];
                infos.reg_value_max = ct[i][7];

                // Ignore the '-1' and '-2' values for min and max, indicating "no boundaries"
                if (infos.reg_value_min < 0)
                {
                    infos.reg_value_min = 0;
                }
                if (infos.reg_value_max < 0)
                {
                    if (ct[i][1] < 5)
                    {
                        infos.reg_value_max = static_cast<int>(pow(2, infos.reg_size*8));
                    }
                    else
                    {
                        infos.reg_value_max = 0xFFFFFFFF;
                    }
                }
                status = 1;
                break; // exit search loop
            }
        }
    }

    return status;
}

int getRegisterName(const int ct[][8], const int reg_index)
{
    int name = -1;

    if (ct != NULL)
    {
        if (reg_index >= 0 && reg_index < static_cast<int>(getRegisterCount(ct)))
        {
            name = ct[reg_index][0];
        }
    }

    return name;
}

int getRegisterTableIndex(const int ct[][8], const int reg_name)
{
    int index = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                index = i;
                break;
            }
        }
    }

    return index;
}

int getRegisterAddr(const int ct[][8], const int reg_name, const int reg_type)
{
    int addr = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                if (reg_type == REGISTER_AUTO)
                {
                    int rom = ct[i][3];
                    int ram = ct[i][4];

                    if (rom != -1)
                        addr = rom;
                    else if (ram != -1)
                        addr = ram;
                }
                else if (reg_type == REGISTER_ROM)
                {
                    addr = ct[i][3];
                }
                else if (reg_type == REGISTER_RAM)
                {
                    addr = ct[i][4];
                }
                break;
            }
        }
    }

    return addr;
}

int getRegisterSize(const int ct[][8], const int reg_name)
{
    int size = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                size = ct[i][1];
                break;
            }
        }
    }

    return size;
}

int getRegisterAccessMode(const int ct[][8], const int reg_name)
{
    int mode = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                mode = ct[i][2];
                break;
            }
        }
    }

    return mode;
}

int getRegisterInitialValue(const int ct[][8], const int reg_name)
{
    int value = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                value = ct[i][5];
                break;
            }
        }
    }

    return value;
}

int getRegisterBounds(const int ct[][8], const int reg_name, int &min, int &max)
{
    int status = -1;

    if (ct != NULL)
    {
        for (unsigned i = 0; i < getRegisterCount(ct); i++)
        {
            if (ct[i][0] == reg_name) // match register name
            {
                min = ct[i][6];
                max = ct[i][7];

                // Ignore the '-1' and '-2' values for min and max, indicating "no boundaries"
                if (min < 0)
                {
                    min = 0;
                }
                if (max < 0)
                {
                    if (ct[i][1] < 5)
                    {
                        max = static_cast<int>(pow(2, (ct[i][1])*8));
                    }
                    else
                    {
                        max = 0xFFFFFFFF;
                    }
                }

                status = 1;
                break;
            }
        }
    }

    return status;
}
