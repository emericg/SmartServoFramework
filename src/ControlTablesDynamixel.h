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
 * \file ControlTablesDynamixel.h
 * \date 24/07/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef CONTROL_TABLES_DYNAMIXEL_H
#define CONTROL_TABLES_DYNAMIXEL_H
/* ************************************************************************** */

#include "Utils.h"

/* ************************************************************************** */

/** \addtogroup ControlTables
 *  @{
 */

/*!
 * \brief AX / DX / RX control table.
 *
 * AX, DX and RX series are using the exact same control table.
 *
 * More details:
 * http://support.robotis.com/en/product/dynamixel/dxl_dx_main.htm
 * http://support.robotis.com/en/product/dynamixel/dxl_ax_main.htm
 * http://support.robotis.com/en/product/dynamixel/dxl_rx_main.htm
 */
const int AXDXRX_control_table[33][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,    1,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,    1,    1,  254 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_MIN_POSITION         , 2, READ_WRITE,  6, -1,    0,    0, 1023 },
    { REG_MAX_POSITION         , 2, READ_WRITE,  8, -1, 1023,    0, 1023 },
    { REG_TEMPERATURE_LIMIT    , 1, READ_WRITE, 11, -1,   65,    0,  150 },
    { REG_VOLTAGE_LOWEST_LIMIT , 1, READ_WRITE, 12, -1,   90,   50,  250 },
    { REG_VOLTAGE_HIGHEST_LIMIT, 1, READ_WRITE, 13, -1,  120,   50,  250 },
    { REG_MAX_TORQUE           , 2, READ_WRITE, 14, -1, 1023,    0, 1023 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 16, -1,    2,    0,    2 },
    { REG_ALARM_LED            , 1, READ_WRITE, 17, -1,   36,    0,  127 },
    { REG_ALARM_SHUTDOWN       , 1, READ_WRITE, 18, -1,   36,    0,  127 },
    // Control table // RAM area
    { REG_TORQUE_ENABLE        , 1, READ_WRITE, -1, 24,    0,    0,    1 },
    { REG_LED                  , 1, READ_WRITE, -1, 25,    0,    0,    1 },
    { REG_CW_COMPLIANCE_MARGIN , 1, READ_WRITE, -1, 26,    0,    0,   255},
    { REG_CCW_COMPLIANCE_MARGIN, 1, READ_WRITE, -1, 27,    0,    0,   255},
    { REG_CW_COMPLIANCE_SLOPE  , 1, READ_WRITE, -1, 28,    0,    2,   128},
    { REG_CCW_COMPLIANCE_SLOPE , 1, READ_WRITE, -1, 29,    0,    2,   128},
    { REG_GOAL_POSITION        , 2, READ_WRITE, -1, 30,   -1,    0, 1023 },
    { REG_GOAL_SPEED           , 2, READ_WRITE, -1, 32,   -1,    0, 1023 },
    { REG_TORQUE_LIMIT         , 2, READ_WRITE, -1, 34,   -1,    0, 1023 },
    { REG_CURRENT_POSITION     , 2, READ_ONLY,  -1, 36,   -1,   -1,   -1 },
    { REG_CURRENT_SPEED        , 2, READ_ONLY,  -1, 38,   -1,   -1,   -1 },
    { REG_CURRENT_LOAD         , 2, READ_ONLY,  -1, 40,   -1,   -1,   -1 },
    { REG_CURRENT_VOLTAGE      , 1, READ_ONLY,  -1, 42,   -1,   -1,   -1 },
    { REG_CURRENT_TEMPERATURE  , 1, READ_ONLY,  -1, 43,   -1,   -1,   -1 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 44,    0,   -1,   -1 },
    { REG_MOVING               , 1, READ_ONLY,  -1, 46,    0,   -1,   -1 },
    { REG_LOCK                 , 1, READ_WRITE, -1, 47,    0,    0,    1 },
    { REG_PUNCH                , 2, READ_WRITE, -1, 48,   32,    0, 1023 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief EX-106 / 106+ control table.
 *
 * More details:
 * http://support.robotis.com/en/product/dynamixel/dxl_ex_main.htm
 */
const int EX_control_table[35][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,  107,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,    1,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,   34,    1,  254 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_MIN_POSITION         , 2, READ_WRITE,  6, -1,    0,    0, 4095 },
    { REG_MAX_POSITION         , 2, READ_WRITE,  8, -1, 4095,    0, 4095 },
    { REG_DRIVE_MODE           , 1, READ_WRITE, 10, -1,    0,    0,    2 },
    { REG_TEMPERATURE_LIMIT    , 1, READ_WRITE, 11, -1,   65,    0,  150 },
    { REG_VOLTAGE_LOWEST_LIMIT , 1, READ_WRITE, 12, -1,  100,   50,  250 },
    { REG_VOLTAGE_HIGHEST_LIMIT, 1, READ_WRITE, 13, -1,  148,   50,  250 },
    { REG_MAX_TORQUE           , 2, READ_WRITE, 14, -1, 1023,    0, 1023 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 16, -1,    2,    0,    2 },
    { REG_ALARM_LED            , 1, READ_WRITE, 17, -1,   36,    0,  127 },
    { REG_ALARM_SHUTDOWN       , 1, READ_WRITE, 18, -1,   36,    0,  127 },
    // Control table // RAM area
    { REG_TORQUE_ENABLE        , 1, READ_WRITE, -1, 24,    0,    0,    1 },
    { REG_LED                  , 1, READ_WRITE, -1, 25,    0,    0,    1 },
    { REG_CW_COMPLIANCE_MARGIN , 1, READ_WRITE, -1, 26,    0,    0,   255},
    { REG_CCW_COMPLIANCE_MARGIN, 1, READ_WRITE, -1, 27,    0,    0,   255},
    { REG_CW_COMPLIANCE_SLOPE  , 1, READ_WRITE, -1, 28,    0,    2,   128},
    { REG_CCW_COMPLIANCE_SLOPE , 1, READ_WRITE, -1, 29,    0,    2,   128},
    { REG_GOAL_POSITION        , 2, READ_WRITE, -1, 30,   -1,    0, 4095 },
    { REG_GOAL_SPEED           , 2, READ_WRITE, -1, 32,   -1,    0, 1023 },
    { REG_TORQUE_LIMIT         , 2, READ_WRITE, -1, 34,   -1,    0, 1023 },
    { REG_CURRENT_POSITION     , 2, READ_ONLY,  -1, 36,   -1,   -1,   -1 },
    { REG_CURRENT_SPEED        , 2, READ_ONLY,  -1, 38,   -1,   -1,   -1 },
    { REG_CURRENT_LOAD         , 2, READ_ONLY,  -1, 40,   -1,   -1,   -1 },
    { REG_CURRENT_VOLTAGE      , 1, READ_ONLY,  -1, 42,   -1,   -1,   -1 },
    { REG_CURRENT_TEMPERATURE  , 1, READ_ONLY,  -1, 43,   -1,   -1,   -1 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 44,    0,   -1,   -1 },
    { REG_MOVING               , 1, READ_ONLY,  -1, 46,    0,   -1,   -1 },
    { REG_LOCK                 , 1, READ_WRITE, -1, 47,    0,    0,    1 },
    { REG_PUNCH                , 2, READ_WRITE, -1, 48,    0,    0, 1023 },
    { REG_CURRENT_CURRENT      , 2, READ_ONLY,  -1, 56,    0,    0, 1023 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief MX control table.
 *
 * More details:
 * http://support.robotis.com/en/product/dynamixel/dxl_mx_main.htm
 */
const int MX_control_table[39][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,    1,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,   34,    1,  254 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_MIN_POSITION         , 2, READ_WRITE,  6, -1,    0,    0, 4095 },
    { REG_MAX_POSITION         , 2, READ_WRITE,  8, -1, 4095,    0, 4095 },
    { REG_DRIVE_MODE           , 1, READ_WRITE, 10, -1,    0,    0,    2 },
    { REG_TEMPERATURE_LIMIT    , 1, READ_WRITE, 11, -1,   65,    0,  150 },
    { REG_VOLTAGE_LOWEST_LIMIT , 1, READ_WRITE, 12, -1,  100,   50,  250 },
    { REG_VOLTAGE_HIGHEST_LIMIT, 1, READ_WRITE, 13, -1,  148,   50,  250 },
    { REG_MAX_TORQUE           , 2, READ_WRITE, 14, -1, 1023,    0, 1023 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 16, -1,    2,    0,    2 },
    { REG_ALARM_LED            , 1, READ_WRITE, 17, -1,   36,    0,  127 },
    { REG_ALARM_SHUTDOWN       , 1, READ_WRITE, 18, -1,   36,    0,  127 },
    { REG_MULTI_TURN_OFFSET    , 2, READ_WRITE, 20, -1,    0,    0, 1024 }, // min max ?
    { REG_RESOLUTION_DIVIDER   , 1, READ_WRITE, 22, -1,    1,    1,    4 },
    // Control table // RAM area
    { REG_TORQUE_ENABLE        , 1, READ_WRITE, -1, 24,    0,    0,    1 },
    { REG_LED                  , 1, READ_WRITE, -1, 25,    0,    0,    1 },
    { REG_D_GAIN               , 1, READ_WRITE, -1, 26,    0,    0,  254 },
    { REG_I_GAIN               , 1, READ_WRITE, -1, 27,    0,    0,  254 },
    { REG_P_GAIN               , 1, READ_WRITE, -1, 28,    0,    0,  254 },
    { REG_GOAL_POSITION        , 2, READ_WRITE, -1, 30,   -1,    0, 4095 },
    { REG_GOAL_SPEED           , 2, READ_WRITE, -1, 32,   -1,    0, 1023 },
    { REG_TORQUE_LIMIT         , 2, READ_WRITE, -1, 34,   -1,    0, 1023 },
    { REG_CURRENT_POSITION     , 2, READ_ONLY,  -1, 36,   -1,   -1,   -1 },
    { REG_CURRENT_SPEED        , 2, READ_ONLY,  -1, 38,   -1,   -1,   -1 },
    { REG_CURRENT_LOAD         , 2, READ_ONLY,  -1, 40,   -1,   -1,   -1 },
    { REG_CURRENT_VOLTAGE      , 1, READ_ONLY,  -1, 42,   -1,   -1,   -1 },
    { REG_CURRENT_TEMPERATURE  , 1, READ_ONLY,  -1, 43,   -1,   -1,   -1 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 44,    0,   -1,   -1 },
    { REG_MOVING               , 1, READ_ONLY,  -1, 46,    0,   -1,   -1 },
    { REG_LOCK                 , 1, READ_WRITE, -1, 47,    0,    0,    1 },
    { REG_PUNCH                , 2, READ_WRITE, -1, 48,    0,    0, 1023 },
    { REG_CURRENT_CURRENT      , 2, READ_ONLY,  -1, 68,    0,    0, 4095 },
    { REG_CONTROL_MODE         , 1, READ_WRITE, -1, 70,    0,    0,    1 },
    { REG_GOAL_TORQUE          , 2, READ_WRITE, -1, 71,    0,    0, 2047 },
    { REG_GOAL_ACCELERATION    , 2, READ_WRITE, -1, 72,    0,    0,  254 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief XL-320 control table.
 *
 * More details:
 * http://support.robotis.com/en/product/dynamixel/xl-320.htm
 */
const int XL320_control_table[32][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,  350,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -1,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,    1,    0,  252 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,    3,    0,    3 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_MIN_POSITION         , 2, READ_WRITE,  6, -1,    0,    0, 1023 },
    { REG_MAX_POSITION         , 2, READ_WRITE,  8, -1, 1023,    0, 1023 },
    { REG_CONTROL_MODE         , 1, READ_WRITE, 11, -1,    2,    1,    2 },
    { REG_TEMPERATURE_LIMIT    , 1, READ_WRITE, 12, -1,   65,    0,  150 },
    { REG_VOLTAGE_LOWEST_LIMIT , 1, READ_WRITE, 13, -1,   60,   50,  250 },
    { REG_VOLTAGE_HIGHEST_LIMIT, 1, READ_WRITE, 14, -1,   90,   50,  250 },
    { REG_MAX_TORQUE           , 2, READ_WRITE, 15, -1, 1023,    0, 1023 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 17, -1,    2,    0,    2 },
    { REG_ALARM_SHUTDOWN       , 1, READ_WRITE, 18, -1,   36,    0,  127 },
    // Control table // RAM area
    { REG_TORQUE_ENABLE        , 1, READ_WRITE, -1, 24,    0,    0,    1 },
    { REG_LED                  , 1, READ_WRITE, -1, 25,    0,    0,    7 },
    { REG_D_GAIN               , 1, READ_WRITE, -1, 27,    0,    0,  254 },
    { REG_I_GAIN               , 1, READ_WRITE, -1, 28,    0,    0,  254 },
    { REG_P_GAIN               , 1, READ_WRITE, -1, 29,   32,    0, 1023 },
    { REG_GOAL_POSITION        , 2, READ_WRITE, -1, 30,   -1,    0, 2047 },
    { REG_GOAL_SPEED           , 2, READ_WRITE, -1, 32,   -1,    0, 1023 },
    { REG_GOAL_TORQUE          , 2, READ_WRITE, -1, 35,   -1,    0,   -1 },
    { REG_CURRENT_POSITION     , 2, READ_ONLY,  -1, 37,   -1,   -1,   -1 },
    { REG_CURRENT_SPEED        , 2, READ_ONLY,  -1, 39,   -1,   -1,   -1 },
    { REG_CURRENT_LOAD         , 2, READ_ONLY,  -1, 41,   -1,   -1,   -1 },
    { REG_CURRENT_VOLTAGE      , 1, READ_ONLY,  -1, 45,   -1,   -1,   -1 },
    { REG_CURRENT_TEMPERATURE  , 1, READ_ONLY,  -1, 46,   -1,   -1,   -1 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 47,    0,   -1,   -1 },
    { REG_MOVING               , 1, READ_ONLY,  -1, 49,    0,   -1,   -1 },
    { REG_HW_ERROR_STATUS      , 1, READ_ONLY,  -1, 50,    0,   -1,   -1 },
    { REG_PUNCH                , 2, READ_WRITE, -1, 51,   32,    0, 1023 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief AX-S1 control table.
 *
 * AX-S1 is a sensor device. The configuration and the communication of this
 * device is similar to AX-12A's, but it is not operated as a servo motor.
 *
 * More details:
 * http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm
 */
const int AXS1_control_table[30][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,  100,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,    1,    1,  254 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 16, -1,    2,    0,    2 },
    // Control table // RAM area
    { REG_IR_DATA_LEFT         , 1, READ_ONLY,  -1, 26,   -1,    0,  255 },
    { REG_IR_DATA_CENTER       , 1, READ_ONLY,  -1, 27,   -1,    0,  255 },
    { REG_IR_DATA_RIGHT        , 1, READ_ONLY,  -1, 28,   -1,    0,  255 },
    { REG_LIGHT_DATA_LEFT      , 1, READ_ONLY,  -1, 29,   -1,    0,  255 },
    { REG_LIGHT_DATA_CENTER    , 1, READ_ONLY,  -1, 30,   -1,    0,  255 },
    { REG_LIGHT_DATA_RIGHT     , 1, READ_ONLY,  -1, 31,   -1,    0,  255 },
    { REG_IR_OBSTACLE_DETECTED , 1, READ_ONLY,  -1, 32,   -1,    0,    7 },
    { REG_LIGHT_DETECTED       , 1, READ_ONLY,  -1, 33,   -1,    0,    1 },
    { REG_SOUND_DATA           , 1, READ_ONLY,  -1, 35,   -1,    0,  255 },
    { REG_SOUND_DATA_MAX_HOLD  , 1, READ_WRITE, -1, 36,   -1,    0,  255 },
    { REG_SOUND_DETECTED_COUNT , 1, READ_WRITE, -1, 37,   -1,    0,  255 },
    { REG_SOUND_DETECTED_TIME  , 2, READ_WRITE, -1, 38,   -1,    0, 65535 },
    { REG_BUZZER_DATA_0        , 1, READ_WRITE, -1, 40,   -1,    0,   51 },
    { REG_BUZZER_DATA_1        , 1, READ_WRITE, -1, 41,   -1,    0,    5 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 44,    0,    0,    1 },
    { REG_IR_REMOCON_ARRIVED   , 1, READ_ONLY,  -1, 46,    0,    0,    2 },
    { REG_LOCK                 , 1, READ_WRITE, -1, 47,    0,    0,    1 },
    { REG_REMOCON_RX_DATA_0    , 1, READ_ONLY,  -1, 48,   -1,    0,  255 },
    { REG_REMOCON_RX_DATA_1    , 1, READ_ONLY,  -1, 49,   -1,    0,  255 },
    { REG_REMOCON_TX_DATA_0    , 1, READ_WRITE, -1, 50,   -1,    0,  255 },
    { REG_REMOCON_TX_DATA_1    , 1, READ_WRITE, -1, 51,   -1,    0,  255 },
    { REG_IR_DETECT_COMPARE    , 1, READ_WRITE, -1, 52,   -1,    0,  255 },
    { REG_LIGHT_DETECT_COMPARE , 1, READ_WRITE, -1, 53,   -1,    0,  255 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief IR Sensor Array control table.
 *
 * AX-S1 is a sensor device. The configuration and the communication type of AX-S1
 * are equal to AX-12A's, but it is not operated by servo motor.
 *
 * More details:
 * http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm
 */
const int IR_ARRAY_control_table[28][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  3, -1,  100,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  4, -1,    1,    1,  254 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  5, -1,  250,    0,  254 },
    { REG_IR_THRESHOLD_1       , 2, READ_WRITE,  6, 48,  784,    0, 65535 },
    { REG_IR_THRESHOLD_2       , 2, READ_WRITE,  8, 50,  784,    0, 65535 },
    { REG_IR_THRESHOLD_3       , 2, READ_WRITE, 10, 52,  784,    0, 65535 },
    { REG_IR_THRESHOLD_4       , 2, READ_WRITE, 12, 54,  784,    0, 65535 },
    { REG_IR_THRESHOLD_5       , 2, READ_WRITE, 14, 56,  784,    0, 65535 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, 16, -1,    2,    0,    2 },
    { REG_IR_THRESHOLD_6       , 2, READ_WRITE, 18, 58,  784,    0, 65535 },
    { REG_IR_THRESHOLD_7       , 2, READ_WRITE, 20, 60,  784,    0, 65535 },
    // Control table // RAM area
    { REG_IR_DATA_1            , 2, READ_ONLY,  -1, 24,   -1,    0, 65535 },
    { REG_IR_DATA_2            , 2, READ_ONLY,  -1, 26,   -1,    0, 65535 },
    { REG_IR_DATA_3            , 2, READ_ONLY,  -1, 28,   -1,    0, 65535 },
    { REG_IR_DATA_4            , 2, READ_ONLY,  -1, 30,   -1,    0, 65535 },
    { REG_IR_DATA_5            , 2, READ_ONLY,  -1, 32,   -1,    0, 65535 },
    { REG_IR_DATA_6            , 2, READ_ONLY,  -1, 34,   -1,    0, 65535 },
    { REG_IR_DATA_7            , 2, READ_ONLY,  -1, 36,   -1,    0, 65535 },
    { REG_BUZZER_DATA_0        , 1, READ_WRITE, -1, 40,   -1,    0,   51 },
    { REG_BUZZER_DATA_1        , 1, READ_WRITE, -1, 41,   -1,    0,    5 },
    { REG_IR_AUTO_THRESHOLD    , 1, READ_WRITE, -1, 42,   -1,    0,    1 },
    { REG_IR_OBSTACLE_DETECTED , 1, READ_ONLY,  -1, 43,   -1,    0,   64 },
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 44,    0,    0,    1 },
    { REG_LOCK                 , 1, READ_WRITE, -1, 47,    0,    0,    1 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/*!
 * \brief This is the WIP control table for Dynamixel PRO servos.
 * All PRO series servos are using the same control table except for L42 (???).
 *
 * More details:
 * http://support.robotis.com/en/product/dynamixel_pro.htm
 */
const int PRO_control_table[49][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER         , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_MODEL_INFORMATION    , 4, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION     , 1, READ_ONLY,   6, -1,   -2,   -1,   -1 },
    { REG_ID                   , 1, READ_WRITE,  7, -1,    1,    0,  253 },
    { REG_BAUD_RATE            , 1, READ_WRITE,  8, -1,    1,    0,    8 },
    { REG_RETURN_DELAY_TIME    , 1, READ_WRITE,  9, -1,  250,    0,  254 },
    { REG_OPERATING_MODE       , 1, READ_WRITE, 11, -1,    3,    0,    4 },
    { REG_HOMING_OFFSET        , 4, READ_WRITE, 13, -1,    0, -2147483647, 2147483647 }, // Min value should be -2147483648 and not 2147483647, but visual studio do not like it so...
    { REG_MOVING_THRESHOLD     , 4, READ_WRITE, 17, -1,   50,    0, 2147483647 },
    { REG_TEMPERATURE_LIMIT    , 1, READ_WRITE, 21, -1,   80,    0,  150 },
    { REG_VOLTAGE_HIGHEST_LIMIT, 2, READ_WRITE, 22, -1,  400,    0,  400 },
    { REG_VOLTAGE_LOWEST_LIMIT , 2, READ_WRITE, 24, -1,  150,    0,  400 },
    { REG_ACCELERATION_LIMIT   , 4, READ_WRITE, 26, -1,   -1,    0,  2147483647 },
    { REG_TORQUE_LIMIT         , 2, READ_WRITE, 30, -1,   -1,    0,  32767 },
    { REG_VELOCITY_LIMIT       , 4, READ_WRITE, 32, -1,   -1,    0,  2147483647 },
    { REG_MAX_POSITION         , 4, READ_WRITE, 36, -1,   -1, -2147483647, 2147483647 },
    { REG_MIN_POSITION         , 4, READ_WRITE, 40, -1,   -1, -2147483647, 2147483647 },
    { REG_EXTERNAL_PORT_MODE_1 , 1, READ_WRITE, 44,  1,    0,    0,    1 },
    { REG_EXTERNAL_PORT_MODE_2 , 1, READ_WRITE, 45,  1,    0,    0,    1 },
    { REG_EXTERNAL_PORT_MODE_3 , 1, READ_WRITE, 46,  1,    0,    0,    1 },
    { REG_EXTERNAL_PORT_MODE_4 , 1, READ_WRITE, 47,  1,    0,    0,    1 },
    { REG_SHUTDOWN             , 1, READ_WRITE, 48, -1,   26,    0,   31 },
    { REG_INDIRECT_ADDRESS_X   , 2, READ_WRITE, 49, -1,  634,  634,  889 }, // X range from 1 (addr 49) to 256 (addr 569)
    // Control table // RAM area
    { REG_TORQUE_ENABLE        , 1, READ_WRITE, -1, 562,    0,    0,    1 },
    { REG_LED_RED              , 1, READ_WRITE, -1, 563,    0,    0,  255 },
    { REG_LED_GREEN            , 1, READ_WRITE, -1, 564,    0,    0,  255 },
    { REG_LED_BLUE             , 1, READ_WRITE, -1, 565,    0,    0,  255 },
    { REG_D_GAIN               , 2, READ_WRITE, -1, 586,   -1,    0, 32767 }, // Not really sure about this PID controls...
    { REG_I_GAIN               , 2, READ_WRITE, -1, 588,   -1,    0, 32767 },
    { REG_P_GAIN               , 2, READ_WRITE, -1, 594,   -1,    0, 32767 },
    { REG_GOAL_POSITION        , 4, READ_WRITE, -1, 596,   -1, -2147483647, 2147483647 },
    { REG_GOAL_VELOCITY        , 4, READ_WRITE, -1, 600,    0, -2147483647, 2147483647 },
    { REG_GOAL_TORQUE          , 2, READ_WRITE, -1, 604,    0,    0, 32767 },
    { REG_GOAL_ACCELERATION    , 4, READ_WRITE, -1, 606,    0, -2147483647, 2147483647 },
    { REG_MOVING               , 1, READ_ONLY,  -1, 610,   -1,    0,     1 },
    { REG_CURRENT_POSITION     , 4, READ_ONLY,  -1, 611,   -1, -2147483647, 2147483647 },
    { REG_CURRENT_VELOCITY     , 4, READ_ONLY,  -1, 615,   -1, -2147483647, 2147483647 },
    { REG_CURRENT_CURRENT      , 2, READ_ONLY,  -1, 621,   -1,   -1, 32767 },
    { REG_CURRENT_VOLTAGE      , 2, READ_ONLY,  -1, 623,   -1,   -1,  400 },
    { REG_CURRENT_TEMPERATURE  , 1, READ_ONLY,  -1, 625,   -1,   -1,  150 },
    { REG_EXTERNAL_PORT_DATA_1 , 2, READ_WRITE  -1, 626,    0,   -1,   -1 },  // May be read only if locked with SERVO_EXTERNAL_PORT_MODE_1
    { REG_EXTERNAL_PORT_DATA_2 , 2, READ_WRITE  -1, 628,    0,   -1,   -1 },
    { REG_EXTERNAL_PORT_DATA_3 , 2, READ_WRITE  -1, 630,    0,   -1,   -1 },
    { REG_EXTERNAL_PORT_DATA_4 , 2, READ_WRITE  -1, 632,    0,   -1,   -1 },
    { REG_INDIRECT_DATA_X      , 1, READ_WRITE  -1, 634,    0,   -1,   -1 },  // X range from 1 (addr 634) to 256 (addr 889)
    { REG_REGISTERED           , 1, READ_ONLY,  -1, 890,    0,   -1,   -1 },
    { REG_STATUS_RETURN_LEVEL  , 1, READ_WRITE, -1, 891,    2,   -1,   -1 },
    { REG_HW_ERROR_STATUS      , 2, READ_ONLY,  -1, 892,    0,   -1,   -1 },
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/** @}*/

/* ************************************************************************** */
#endif /* CONTROL_TABLES_DYNAMIXEL_H */
