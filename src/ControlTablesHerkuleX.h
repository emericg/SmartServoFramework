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
 * \file ControlTablesHerkuleX.h
 * \date 24/07/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef CONTROL_TABLES_HERKULEX_H
#define CONTROL_TABLES_HERKULEX_H
/* ************************************************************************** */

#include "Utils.h"

/* ************************************************************************** */

/** \addtogroup ControlTables
 *  @{
 */

/*!
 * \brief DRS-0101 / DRS-0201 servo series control table.
 *
 * More details:
 * - http://hovis.co.kr/guide/herkulex_eng.html
 * - http://www.dongburobot.com/jsp/cms/view.jsp?code=100782
 */
const int DRS0101_control_table[50][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER            , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION        , 2, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_BAUD_RATE               , 1, READ_WRITE,  4, -1,   16,    2,   34 },
// reserved
    // Control table // EEPROM and RAM area
    { REG_ID                      , 1, READ_WRITE,  6,  0,  253,    0,  253 },
    { REG_STATUS_RETURN_LEVEL     , 1, READ_WRITE,  7,  1,    1,    0,    2 }, // ACK_POLICY
    { REG_ALARM_LED               , 1, READ_WRITE,  8,  2,  127,    0,  127 }, // ALARM_LED_POLICY
    { REG_ALARM_SHUTDOWN          , 1, READ_WRITE,  9,  3,   53,    0,  127 }, // TORQUE_POLICY
// reserved
    { REG_TEMPERATURE_LIMIT       , 1, READ_WRITE, 11,  5,  222,    0,  254 }, // MAX_TEMPERATURE
    { REG_VOLTAGE_LOWEST_LIMIT    , 1, READ_WRITE, 12,  6,   91,    0,  254 }, // MIN_VOLTAGE
    { REG_VOLTAGE_HIGHEST_LIMIT   , 1, READ_WRITE, 13,  7,  137,    0,  254 }, // MAX_VOLTAGE
    { REG_ACCEL_RATIO             , 1, READ_WRITE, 14,  8,   25,    0,   50 },
    { REG_MAX_ACCEL_TIME          , 1, READ_WRITE, 15,  9,   45,    0,  254 },
    { REG_DEAD_ZONE               , 1, READ_WRITE, 16, 10,    0,    0,  254 },
    { REG_SATURATOR_OFFSET        , 1, READ_WRITE, 17, 11,    0,    0,  254 },
    { REG_SATURATOR_SLOPE         , 2, READ_WRITE, 18, 12,    0,    0, 32767 },
    { REG_PWM_OFFSET              , 1, READ_WRITE, 20, 14,    0, -128,  127 },
    { REG_PWM_MIN                 , 1, READ_WRITE, 21, 15,    0,    0,  254 },
    { REG_PWM_MAX                 , 2, READ_WRITE, 22, 16, 1023,    0, 1023 },
    { REG_PWM_OVERLOAD_THRESHOLD  , 2, READ_WRITE, 24, 18,    0,    0, 32766 },
    { REG_MIN_POSITION            , 2, READ_WRITE, 26, 20,   21,    0, 1023 },
    { REG_MAX_POSITION            , 2, READ_WRITE, 28, 22, 1002,    0, 1023 },
    { REG_P_GAIN                  , 2, READ_WRITE, 30, 24,  440,    0, 32767 }, // POSITION_KP // Proportional gain
    { REG_D_GAIN                  , 2, READ_WRITE, 32, 26, 8000,    0, 32767 }, // POSITION_KD // Derivative gain
    { REG_I_GAIN                  , 2, READ_WRITE, 34, 28,    0,    0, 32767 }, // POSITION_KI // Integral gain
    { REG_POS_FEED_FRW_1st_GAIN   , 2, READ_WRITE, 36, 30,    0,    0, 32767 }, // Position Feed forward 1st gain
    { REG_POS_FEED_FRW_2nd_GAIN   , 2, READ_WRITE, 38, 32,    0,    0, 32767 }, // Position Feed forward 2nd gain
// reserved
    { REG_LED_BLINKING            , 1, READ_WRITE, 44, 38,   45,    0,  254 },
    { REG_ADC_FAULT_CHECK_PRD     , 1, READ_WRITE, 45, 39,   45,    0,  254 },
    { REG_PKT_GARBAGE_CHECK_PRD   , 1, READ_WRITE, 46, 40,   18,    0,  254 },
    { REG_STOP_DETECTION_PRD      , 1, READ_WRITE, 47, 41,   27,    0,  254 },
    { REG_OVERLOAD_DETECTION_PRD  , 1, READ_WRITE, 48, 42,  150,    0,  254 },
    { REG_STOP_THRESHOLD          , 1, READ_WRITE, 49, 43,    3,    0,  254 },
// reserved
    { REG_INPOSITION_MARGIN       , 1, READ_WRITE, 50, 44,    3,    0,  254 },
    { REG_CALIBRATION_DIFFERENCE  , 1, READ_WRITE, 53, 47,    0, -128,  127 },
    // Control table // RAM area
    { REG_STATUS_ERROR            , 1, READ_WRITE, -1, 48,   -1,    0,  127 },
    { REG_STATUS_DETAIL           , 1, READ_WRITE, -1, 49,   -1,    0,  127 },
// reserved
    { REG_TORQUE_ENABLE           , 1, READ_WRITE, -1, 52,   -1,    0,  254 }, // TORQUE_CONTROL
    { REG_LED                     , 1, READ_WRITE, -1, 53,   -1,    0,    7 }, // LED_CONTROL
    { REG_CURRENT_VOLTAGE         , 1, READ_ONLY,  -1, 54,   -1,    0,  254 },
    { REG_CURRENT_TEMPERATURE     , 1, READ_ONLY,  -1, 55,   -1,    0,  254 },
    { REG_CURRENT_CONTROL_MODE    , 1, READ_ONLY,  -1, 56,   -1,    0,    1 },
    { REG_TICK                    , 1, READ_ONLY,  -1, 57,   -1,    0,  255 },
    { REG_CALIBRATED_POSITION     , 2, READ_ONLY,  -1, 58,   -1,   -1,   -1 },
    { REG_ABSOLUTE_POSITION       , 2, READ_ONLY,  -1, 60,   -1,   -1,   -1 },
    { REG_DIFFERENTIAL_POSITION   , 2, READ_ONLY,  -1, 62,   -1,   -1,   -1 },
    { REG_PWM                     , 2, READ_ONLY,  -1, 64,   -1,   -1,   -1 },
// reserved
    { REG_ABSOLUTE_GOAL_POSITION  , 2, READ_ONLY,  -1, 68,   -1,   -1,   -1 },
    { REG_GOAL_TRAJECTORY         , 2, READ_ONLY,  -1, 70,   -1,   -1,   -1 }, // ABSOLUTE DESIRED TRAJECTORY POSITION
    { REG_GOAL_VELOCITY           , 2, READ_ONLY,  -1, 72,   -1,   -1,   -1 }, // DESIRED VELOCITY
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/* ************************************************************************** */

/*!
 * \brief DRS-0401 / DRS-0601 servo series control table.
 *
 * More details:
 * - http://hovis.co.kr/guide/herkulex_eng.html
 * - http://www.dongburobot.com/jsp/cms/view.jsp?code=100782
 */
const int DRS0x01_control_table[50][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER            , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION        , 2, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_BAUD_RATE               , 1, READ_WRITE,  4, -1,   16,    2,   34 },
// reserved
    // Control table // EEPROM and RAM area
    { REG_ID                      , 1, READ_WRITE,  6,  0,  253,    0,  253 },
    { REG_STATUS_RETURN_LEVEL     , 1, READ_WRITE,  7,  1,    1,    0,    2 }, // ACK_POLICY
    { REG_ALARM_LED               , 1, READ_WRITE,  8,  2,  127,    0,  127 }, // ALARM_LED_POLICY
    { REG_ALARM_SHUTDOWN          , 1, READ_WRITE,  9,  3,   53,    0,  127 }, // TORQUE_POLICY
// reserved
    { REG_TEMPERATURE_LIMIT       , 1, READ_WRITE, 11,  5,  222,    0,  254 }, // MAX_TEMPERATURE
    { REG_VOLTAGE_LOWEST_LIMIT    , 1, READ_WRITE, 12,  6,   91,    0,  254 }, // MIN_VOLTAGE
    { REG_VOLTAGE_HIGHEST_LIMIT   , 1, READ_WRITE, 13,  7,  137,    0,  254 }, // MAX_VOLTAGE
    { REG_ACCEL_RATIO             , 1, READ_WRITE, 14,  8,   25,    0,   50 },
    { REG_MAX_ACCEL_TIME          , 1, READ_WRITE, 15,  9,   45,    0,  254 },
    { REG_DEAD_ZONE               , 1, READ_WRITE, 16, 10,    0,    0,  254 },
    { REG_SATURATOR_OFFSET        , 1, READ_WRITE, 17, 11,    0,    0,  254 },
    { REG_SATURATOR_SLOPE         , 2, READ_WRITE, 18, 12,    0,    0, 32767 },
    { REG_PWM_OFFSET              , 1, READ_WRITE, 20, 14,    0, -128,  127 },
    { REG_PWM_MIN                 , 1, READ_WRITE, 21, 15,    0,    0,  254 },
    { REG_PWM_MAX                 , 2, READ_WRITE, 22, 16, 1023,    0, 1023 },
    { REG_PWM_OVERLOAD_THRESHOLD  , 2, READ_WRITE, 24, 18,    0,    0, 32766 },
    { REG_MIN_POSITION            , 2, READ_WRITE, 26, 20,   21,    0, 1023 },
    { REG_MAX_POSITION            , 2, READ_WRITE, 28, 22, 1002,    0, 1023 },
    { REG_P_GAIN                  , 2, READ_WRITE, 30, 24,  440,    0, 32767 }, // POSITION_KP // Proportional gain
    { REG_D_GAIN                  , 2, READ_WRITE, 32, 26, 8000,    0, 32767 }, // POSITION_KD // Derivative gain
    { REG_I_GAIN                  , 2, READ_WRITE, 34, 28,    0,    0, 32767 }, // POSITION_KI // Integral gain
    { REG_POS_FEED_FRW_1st_GAIN   , 2, READ_WRITE, 36, 30,    0,    0, 32767 }, // Position Feed forward 1st gain
    { REG_POS_FEED_FRW_2nd_GAIN   , 2, READ_WRITE, 38, 32,    0,    0, 32767 }, // Position Feed forward 2nd gain
// reserved
    { REG_LED_BLINKING            , 1, READ_WRITE, 44, 38,   45,    0,  254 },
    { REG_ADC_FAULT_CHECK_PRD     , 1, READ_WRITE, 45, 39,   45,    0,  254 },
    { REG_PKT_GARBAGE_CHECK_PRD   , 1, READ_WRITE, 46, 40,   18,    0,  254 },
    { REG_STOP_DETECTION_PRD      , 1, READ_WRITE, 47, 41,   27,    0,  254 },
    { REG_OVERLOAD_DETECTION_PRD  , 1, READ_WRITE, 48, 42,  150,    0,  254 },
    { REG_STOP_THRESHOLD          , 1, READ_WRITE, 49, 43,    3,    0,  254 },
// reserved
    { REG_INPOSITION_MARGIN       , 1, READ_WRITE, 50, 44,    3,    0,  254 },
    { REG_CALIBRATION_DIFFERENCE  , 2, READ_WRITE, 52, 46,    0, -255,  255 },
    // Control table // RAM area
    { REG_STATUS_ERROR            , 1, READ_WRITE, -1, 48,   -1,    0,  127 },
    { REG_STATUS_DETAIL           , 1, READ_WRITE, -1, 49,   -1,    0,  127 },
// reserved
    { REG_TORQUE_ENABLE           , 1, READ_WRITE, -1, 52,   -1,    0,  254 }, // TORQUE_CONTROL
    { REG_LED                     , 1, READ_WRITE, -1, 53,   -1,    0,    7 }, // LED_CONTROL
    { REG_CURRENT_VOLTAGE         , 1, READ_ONLY,  -1, 54,   -1,    0,  254 },
    { REG_CURRENT_TEMPERATURE     , 1, READ_ONLY,  -1, 55,   -1,    0,  254 },
    { REG_CURRENT_CONTROL_MODE    , 1, READ_ONLY,  -1, 56,   -1,    0,    1 },
    { REG_TICK                    , 1, READ_ONLY,  -1, 57,   -1,    0,  255 },
    { REG_CALIBRATED_POSITION     , 2, READ_ONLY,  -1, 58,   -1,   -1,   -1 },
    { REG_ABSOLUTE_POSITION       , 2, READ_ONLY,  -1, 60,   -1,   -1,   -1 },
    { REG_DIFFERENTIAL_POSITION   , 2, READ_ONLY,  -1, 62,   -1,   -1,   -1 },
    { REG_PWM                     , 2, READ_ONLY,  -1, 64,   -1,   -1,   -1 },
// reserved
    { REG_ABSOLUTE_GOAL_POSITION  , 2, READ_ONLY,  -1, 68,   -1,   -1,   -1 },
    { REG_GOAL_TRAJECTORY         , 2, READ_ONLY,  -1, 70,   -1,   -1,   -1 }, // ABSOLUTE DESIRED TRAJECTORY POSITION
    { REG_GOAL_VELOCITY           , 2, READ_ONLY,  -1, 72,   -1,   -1,   -1 }, // DESIRED VELOCITY
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/* ************************************************************************** */

/*!
 * \brief DRS-0402 / DRS-0602 servo series control table.
 *
 * More details:
 * - http://hovis.co.kr/guide/herkulex_eng.html
 * - http://www.dongburobot.com/jsp/cms/view.jsp?code=100782
 */
const int DRS0x02_control_table[54][8] =
{
    // Instruction // size // RW Access // ROM addr // RAM addr // initial value // min value // max value

    // Control table // EEPROM area
    { REG_MODEL_NUMBER            , 2, READ_ONLY,   0, -1,   -2,   -1,   -1 },
    { REG_FIRMWARE_VERSION        , 2, READ_ONLY,   2, -1,   -2,   -1,   -1 },
    { REG_BAUD_RATE               , 1, READ_WRITE,  4, -1,   16,    2,   34 },
// reserved
    // Control table // EEPROM and RAM area
    { REG_ID                      , 1, READ_WRITE,  6,  0,  253,    0,  253 },
    { REG_STATUS_RETURN_LEVEL     , 1, READ_WRITE,  7,  1,    1,    0,    2 }, // ACK_POLICY
    { REG_ALARM_LED               , 1, READ_WRITE,  8,  2,  127,    0,  127 }, // ALARM_LED_POLICY
    { REG_ALARM_SHUTDOWN          , 1, READ_WRITE,  9,  3,   53,    0,  127 }, // TORQUE_POLICY
// reserved
    { REG_TEMPERATURE_LIMIT       , 1, READ_WRITE, 11,  5,  222,    0,  254 }, // MAX_TEMPERATURE
    { REG_VOLTAGE_LOWEST_LIMIT    , 1, READ_WRITE, 12,  6,   91,    0,  254 }, // MIN_VOLTAGE
    { REG_VOLTAGE_HIGHEST_LIMIT   , 1, READ_WRITE, 13,  7,  137,    0,  254 }, // MAX_VOLTAGE
    { REG_ACCEL_RATIO             , 1, READ_WRITE, 14,  8,   25,    0,   50 },
    { REG_MAX_ACCEL_TIME          , 1, READ_WRITE, 15,  9,   45,    0,  254 },
    { REG_DEAD_ZONE               , 1, READ_WRITE, 16, 10,    0,    0,  254 },
    { REG_SATURATOR_OFFSET        , 1, READ_WRITE, 17, 11,    0,    0,  254 },
    { REG_SATURATOR_SLOPE         , 2, READ_WRITE, 18, 12,    0,    0, 32767 },
    { REG_PWM_OFFSET              , 1, READ_WRITE, 20, 14,    0, -128,  127 },
    { REG_PWM_MIN                 , 1, READ_WRITE, 21, 15,    0,    0,  254 },
    { REG_PWM_MAX                 , 2, READ_WRITE, 22, 16, 1023,    0, 1023 },
    { REG_PWM_OVERLOAD_THRESHOLD  , 2, READ_WRITE, 24, 18,    0,    0, 32766 },
    { REG_MIN_POSITION            , 2, READ_WRITE, 26, 20,   21,    0, 1023 },
    { REG_MAX_POSITION            , 2, READ_WRITE, 28, 22, 1002,    0, 1023 },
    { REG_P_GAIN                  , 2, READ_WRITE, 30, 24,  440,    0, 32767 }, // POSITION_KP // Proportional gain
    { REG_D_GAIN                  , 2, READ_WRITE, 32, 26, 8000,    0, 32767 }, // POSITION_KD // Derivative gain
    { REG_I_GAIN                  , 2, READ_WRITE, 34, 28,    0,    0, 32767 }, // POSITION_KI // Integral gain
    { REG_POS_FEED_FRW_1st_GAIN   , 2, READ_WRITE, 36, 30,    0,    0, 32767 }, // Position Feed forward 1st gain
    { REG_POS_FEED_FRW_2nd_GAIN   , 2, READ_WRITE, 38, 32,    0,    0, 32767 }, // Position Feed forward 2nd gain
    { REG_VELOCITY_KP             , 2, READ_ONLY , 40, 34,  100,    0, 32767 }, // Velocity controller P Gain, Actual P Gain=(Velocity Kp)/64
    { REG_VELOCITY_KI             , 2, READ_ONLY , 42, 36,12000,    0, 32767 }, // Velocity controller I Gain, Actual I Gain=(Velocity Ki)/16384
    { REG_LED_BLINKING            , 1, READ_WRITE, 44, 38,   45,    0,  254 },
    { REG_ADC_FAULT_CHECK_PRD     , 1, READ_WRITE, 45, 39,   45,    0,  254 },
    { REG_PKT_GARBAGE_CHECK_PRD   , 1, READ_WRITE, 46, 40,   18,    0,  254 },
    { REG_STOP_DETECTION_PRD      , 1, READ_WRITE, 47, 41,   27,    0,  254 },
    { REG_OVERLOAD_DETECTION_PRD  , 1, READ_WRITE, 48, 42,  150,    0,  254 },
    { REG_STOP_THRESHOLD          , 1, READ_WRITE, 49, 43,    3,    0,  254 },
// reserved
    { REG_INPOSITION_MARGIN       , 1, READ_WRITE, 50, 44,    3,    0,  254 },
    { REG_CALIBRATION_DIFFERENCE  , 2, READ_WRITE, 52, 46,    0, -255,  255 },
    // Control table // RAM area
    { REG_STATUS_ERROR            , 1, READ_WRITE, -1, 48,   -1,    0,  127 },
    { REG_STATUS_DETAIL           , 1, READ_WRITE, -1, 49,   -1,    0,  127 },
    { REG_AUX_1                   , 1, READ_WRITE, -1, 50,   -1,    0,    6 },
// reserved
    { REG_TORQUE_ENABLE           , 1, READ_WRITE, -1, 52,   -1,    0,  254 }, // TORQUE_CONTROL
    { REG_LED                     , 1, READ_WRITE, -1, 53,   -1,    0,    7 }, // LED_CONTROL
    { REG_CURRENT_VOLTAGE         , 1, READ_ONLY,  -1, 54,   -1,    0,  254 },
    { REG_CURRENT_TEMPERATURE     , 1, READ_ONLY,  -1, 55,   -1,    0,  254 },
    { REG_CURRENT_CONTROL_MODE    , 1, READ_ONLY,  -1, 56,   -1,    0,    1 },
    { REG_TICK                    , 1, READ_ONLY,  -1, 57,   -1,    0,  255 },
    { REG_CALIBRATED_POSITION     , 2, READ_ONLY,  -1, 58,   -1,   -1,   -1 },
    { REG_ABSOLUTE_POSITION       , 2, READ_ONLY,  -1, 60,   -1,   -1,   -1 }, // Absolute postion Raw Date
    { REG_DIFFERENTIAL_POSITION   , 2, READ_ONLY,  -1, 62,   -1,   -1,   -1 },
    { REG_PWM                     , 2, READ_ONLY,  -1, 64,   -1,   -1,   -1 },
    { REG_ABSOLUTE_2nd_POSITION   , 2, READ_ONLY,  -1, 66,   -1,   -1,   -1 }, // Potentiometer absolute position Raw Data
    { REG_ABSOLUTE_GOAL_POSITION  , 2, READ_ONLY,  -1, 68,   -1,   -1,   -1 },
    { REG_GOAL_TRAJECTORY         , 2, READ_ONLY,  -1, 70,   -1,   -1,   -1 }, // ABSOLUTE DESIRED TRAJECTORY POSITION
    { REG_GOAL_VELOCITY           , 2, READ_ONLY,  -1, 72,   -1,   -1,   -1 }, // DESIRED VELOCITY
    { 999, 999, 999, 999, 999, 999, 999, 999 },
};

/** @}*/

/* ************************************************************************** */
#endif /* CONTROL_TABLES_HERKULEX_H */
