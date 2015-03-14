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
 * \file HerkuleXTools.cpp
 * \date 07/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "HerkuleXTools.h"
#include <iostream>

std::string hkx_get_model_name(const int model_number)
{
    std::string name;

    switch (model_number & 0x0000FFFF)
    {
    case 0x0101:
        name = "DRS-0101";
        break;
    case 0x0201:
        name = "DRS-0201";
        break;
    case 0x0401:
        name = "DRS-0401";
        break;
    case 0x0402:
        name = "DRS-0402";
        break;
    case 0x0601:
        name = "DRS-0601";
        break;
    case 0x0602:
        name = "DRS-0602";
        break;

    default:
        name = "Unknown";
        break;
    }

    return name;
}

void hkx_get_model_infos(const int model_number, int &servo_serie, int &servo_model)
{
    switch (model_number & 0x0000FFFF)
    {
    case 0x0101:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0101;
        break;
    case 0x0201:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0201;
        break;
    case 0x0401:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0401;
        break;
    case 0x0402:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0402;
        break;
    case 0x0601:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0601;
        break;
    case 0x0602:
        servo_serie = SERVO_DRS;
        servo_model = SERVO_DRS_0602;
        break;

    default:
        servo_serie = SERVO_UNKNOWN;
        servo_model = SERVO_UNKNOWN;
        break;
    }
}

int hkx_get_servo_model(const int model_number)
{
    int servo_serie, servo_model;
    hkx_get_model_infos(model_number, servo_serie, servo_model);

    return servo_model;
}

int hkx_get_baudrate(const int baudnum, const int servo_serie)
{
    int baudRate = 115200;

    if (servo_serie == 0)
    {
        std::cerr << "Unknown servo serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
    }
    else if (servo_serie >= SERVO_DRS)
    {
        switch (baudnum)
        {
        case 0x01:
            baudRate = 1000000; // Not available on DRS-0101 and DRS-0201
            break;
        case 0x02:
            baudRate = 666666;
            break;
        case 0x03:
            baudRate = 500000;
            break;
        case 0x04:
            baudRate = 400000;
            break;
        case 0x07:
            baudRate = 250000;
            break;
        case 0x09:
            baudRate = 200000;
            break;
        case 0x10:
            baudRate = 115200;
            break;
        case 0x22:
            baudRate = 57600;
            break;
        default:
            std::cerr << "Invalid baudnum (" << baudnum << ") for DRS serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
            break;
        }
    }
    else
    {
        std::cerr << "Unsupported HerkuleX servo serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
    }

    return baudRate;
}
