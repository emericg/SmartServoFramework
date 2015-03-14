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
 * \file DynamixelTools.cpp
 * \date 11/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "DynamixelTools.h"
#include <iostream>

std::string dxl_get_model_name(const int model_number)
{
    std::string name;

    switch (model_number & 0x0000FFFF)
    {
    case 0x000C:
        name = "AX-12A";
        break;
    case 0x012C:
        name = "AX-12W";
        break;
    case 0x0012:
        name = "AX-18A";
        break;

    case 0x0071:
        name = "DX-113";
        break;
    case 0x0074:
        name = "DX-116";
        break;
    case 0x0075:
        name = "DX-117";
        break;

    case 0x000A:
        name = "RX-10";
        break;
    case 0x0018:
        name = "RX-24F";
        break;
    case 0x001C:
        name = "RX-28";
        break;
    case 0x0040:
        name = "RX-64";
        break;

    case 0x006A:
        name = "EX-106";
        break;
    case 0x006B:
        name = "EX-106+";
        break;

    case 0x0168:
        name = "MX-12W";
        break;
    case 0x001D:
        name = "MX-28";
        break;
    case 0x0136:
        name = "MX-64";
        break;
    case 0x0140:
        name = "MX-106";
        break;

    case 0x015E:
        name = "XL-320";
        break;

    case 0x0013:
        name = "AX-S1";
        break;
    case 0x014A:
        name = "IR Sensor Array";
        break;

    default:
        name = "Unknown";
        break;
    }

    return name;
}

void dxl_get_model_infos(const int model_number, int &servo_serie, int &servo_model)
{
    switch (model_number & 0x0000FFFF)
    {
    case 0x000C:
        servo_serie = SERVO_AX;
        servo_model = SERVO_AX12A;
        break;
    case 0x012C:
        servo_serie = SERVO_AX;
        servo_model = SERVO_AX12W;
        break;
    case 0x0012:
        servo_serie = SERVO_AX;
        servo_model = SERVO_AX18A;
        break;

    case 0x0071:
        servo_serie = SERVO_DX;
        servo_model = SERVO_DX113;
        break;
    case 0x0074:
        servo_serie = SERVO_DX;
        servo_model = SERVO_DX116;
        break;
    case 0x0075:
        servo_serie = SERVO_DX;
        servo_model = SERVO_DX117;
        break;

    case 0x000A:
        servo_serie = SERVO_RX;
        servo_model = SERVO_RX10;
        break;
    case 0x0018:
        servo_serie = SERVO_RX;
        servo_model = SERVO_RX24F;
        break;
    case 0x001C:
        servo_serie = SERVO_RX;
        servo_model = SERVO_RX28;
        break;
    case 0x0040:
        servo_serie = SERVO_RX;
        servo_model = SERVO_RX64;
        break;

    case 0x006A:
        servo_serie = SERVO_EX;
        servo_model = SERVO_EX106;
        break;
    case 0x006B:
        servo_serie = SERVO_EX;
        servo_model = SERVO_EX106p;
        break;

    case 0x0168:
        servo_serie = SERVO_MX;
        servo_model = SERVO_MX12W;
        break;
    case 0x001D:
        servo_serie = SERVO_MX;
        servo_model = SERVO_MX28;
        break;
    case 0x0136:
        servo_serie = SERVO_MX;
        servo_model = SERVO_MX64;
        break;
    case 0x0140:
        servo_serie = SERVO_MX;
        servo_model = SERVO_MX106;
        break;

    case 0x015E:
        servo_serie = SERVO_XL;
        servo_model = SERVO_XL320;
        break;

    case 0x0013:
        servo_serie = SENSOR_DYNAMIXEL;
        servo_model = SENSOR_AXS1;
        break;
    case 0x014A:
        servo_serie = SENSOR_DYNAMIXEL;
        servo_model = SENSOR_IR_ARRAY;
        break;

    default:
        servo_serie = SERVO_UNKNOWN;
        servo_model = SERVO_UNKNOWN;
        break;
    }
}

int dxl_get_servo_model(const int model_number)
{
    int servo_serie, servo_model;
    dxl_get_model_infos(model_number, servo_serie, servo_model);

    return servo_model;
}

int dxl_get_baudrate(const int baudnum, const int servo_serie)
{
    int baudRate = 1000000;

    if (servo_serie == 0)
    {
        std::cerr << "Unknown servo serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
    }
    else if (servo_serie >= SERVO_PRO) // Dynamixel PRO serie
    {
        switch (baudnum)
        {
        case 0:
            baudRate = 2400;
            break;
        case 1:
            baudRate = 57600;
            break;
        case 2:
            baudRate = 115200;
            break;
        case 3:
            baudRate = 1000000;
            break;
        case 4:
            baudRate = 2000000;
            break;
        case 5:
            baudRate = 3000000;
            break;
        case 6:
            baudRate = 4000000;
            break;
        case 7:
            baudRate = 4500000;
            break;
        case 8:
            baudRate = 10500000;
            break;
        default:
            std::cerr << "Invalid baudnum (" << baudnum << ") for PRO serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
            break;
        }
    }
    else if (servo_serie >= SERVO_XL) // Dynamixel XL-320
    {
        switch (baudnum)
        {
        case 0:
            baudRate = 9600;
            break;
        case 1:
            baudRate = 57600;
            break;
        case 2:
            baudRate = 115200;
            break;
        case 3:
            baudRate = 1000000;
            break;
        default:
            std::cerr << "Invalid baudnum (" << baudnum << ") for XL serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
            break;
        }
    }
    else if (servo_serie >= SERVO_MX) // Dynamixel MX serie
    {
        if (baudnum > 0 && baudnum < 255)
        {
            if (baudnum < 250)
            {
                baudRate = static_cast<int>(2000000.0 / static_cast<double>(baudnum + 1));
            }
            else if (baudnum < 255) // Special 'baudnum' values from high end servos
            {
                if (baudnum == 250)
                {
                    baudRate = 2250000;
                }
                else if (baudnum == 251)
                {
                    baudRate = 2500000;
                }
                else // if (baud >= 252)
                {
                    baudRate = 3000000;
                }
            }
        }
        else
        {
            std::cerr << "Invalid baudnum (" << baudnum << ") for MX serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
        }
    }
    else if (servo_serie >= SERVO_AX) // Dynamixel AX / DX / EX / RX series
    {
        if (baudnum > 0 && baudnum < 255)
        {
            baudRate = static_cast<int>(2000000.0 / static_cast<double>(baudnum + 1));
        }
        else
        {
            std::cerr << "Invalid baudnum (" << baudnum << ") for AX serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
        }
    }
    else
    {
        std::cerr << "Unsupported Dynamixel servo serie, using default baudrate of: '" << baudRate << "' bps" << std::endl;
    }

    // Force minimum baudrate to 2400 bps if needed
    if (baudRate < 2400)
    {
        baudRate = 2400;
        std::cerr << "Baudrate value (" << baudRate << ") is too low for Dynamixel devices, using minimum baudrate of 2400 bps." << std::endl;
    }

    return baudRate;
}
