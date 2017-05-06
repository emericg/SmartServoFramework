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
 * \file Utils.h
 * \date 08/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef UTILS_H
#define UTILS_H
/* ************************************************************************** */

// Doxygen documentation groups:

/*!
 * \defgroup SimpleAPIs Simple APIs
 *
 * Use this API to easily get/set values to your servos by sending simple
 * synchronous instructions, then waiting for the answers!
 */

/*!
 * \defgroup ManagedAPIs Managed APIs
 *
 * Setup a controller and attach servo instances to it. Manipulate servo objects
 * and let the controller synchronize (at a fixed frequency) its "virtual" register
 * values with the real servo devices using a background thread.
 *
 * Beware: this API is more complex to master, and not entirely stable yet ;-)
 */

/*!
 * \defgroup ControlTables Low level devices support
 */

/*!
 * \defgroup Tools Tools
 */

/* ************************************************************************** */

/** \addtogroup Tools
 *  @{
 */

/*!
 * \brief Serial communication protocol.
 */
enum ServoProtocol
{
    PROTOCOL_UNKNOWN= 0,    //!<

    PROTOCOL_DXLv1  = 1,    //!<
    PROTOCOL_DXLv2  = 2,    //!<
    PROTOCOL_HKX    = 3,    //!<
};

/*!
 * \brief Broadcast address.
 *
 * If an instruction packet is sent to the broadcast ID, all linked Dynamixels
 * or HerkuleX devices will execute the instruction. No status packet is returned
 * on broadcast operations.
 */
#define BROADCAST_ID          (254)

/*!
 * \brief Ping response structure, filled by the "PING" or "STAT" function.
 */
struct PingResponse
{
    int model_number;
    int firmware_version;
};

/*!
 * \brief The AckPolicy_e enum indicate how a device will answer to an incoming packet.
 *
 * The functionnality is called "Status Return Level" for Dynamixel and "Ack Policy"
 * for HerkuleX devices.
 * It can be very usefull to disable status packet, at least to everything but
 * "READ" commands to avoid overloading your serial link.
 */
enum AckPolicy_e
{
    ACK_DEFAULT    = -1,   //!< Use default policy for the device (ACK_REPLY_ALL for Dynamixel, ACK_REPLY_READ for HerkuleX)

    ACK_NO_REPLY   = 0,    //!< Status packets are disabled (except to Dynamixel "PING" or HerkuleX "STAT" commands)
    ACK_REPLY_READ = 1,    //!< Status packets are sent only to "READ" commands
    ACK_REPLY_ALL  = 2     //!< Status packets are sent to every commands
};

/*!
 * \brief Factory reset settings (only work with Dynamixel protocol v2 or HerkuleX devices).
 */
enum ResetOptions_e
{
    RESET_ALL                       = 0xFF, //!< Reset all values to factory default
    RESET_ALL_EXCEPT_ID             = 0x01, //!< Reset all values except ID
    RESET_ALL_EXCEPT_ID_BAUDRATE    = 0x02  //!< Reset all values except ID and baud rate
};

/*!
 * \brief Led color enum.
 *
 * Dynamixel AX / RX / EX / MX servos only support RED color.
 * Only Dynamixel XL-320 and PRO supports RGB and composite colors.
 * HerkuleX support separate red, green and blue leds but with a different order (G=0x01, B=0x02, R=0x04).
 */
enum LedColors_e
{
    // RGB LEDs
    LED_RED     = 0x01,
    LED_GREEN   = 0x02,
    LED_BLUE    = 0x04,

    // Composite colors, only for Dynamixel XL-320 and PRO
    LED_WHITE       = LED_RED | LED_GREEN | LED_BLUE,
    LED_PINK        = LED_RED | LED_BLUE,
    LED_SOFT_BLUE   = LED_GREEN | LED_BLUE,
    LED_YELLOW      = LED_RED | LED_GREEN,
};

/* ************************************************************************** */

/*!
 * \brief Enum to define the various servos (and sensors) brands, series, and models supported by the framework.
 */
enum ServoDevices_e
{
    SERVO_UNKNOWN = 0,

    // Dynamixel servo devices:
    SERVO_DYNAMIXEL      = 1,

        SERVO_AX         = 10,
            SERVO_AX12A  = 11,
            SERVO_AX12W  = 12,
            SERVO_AX18A  = 13,

        SERVO_DX         = 20,
            SERVO_DX113  = 21,
            SERVO_DX116  = 22,
            SERVO_DX117  = 23,

        SERVO_RX         = 30,
            SERVO_RX10   = 31,
            SERVO_RX24F  = 32,
            SERVO_RX28   = 33,
            SERVO_RX64   = 34,

        SERVO_EX         = 40,
            SERVO_EX106  = 41,
            SERVO_EX106p = 42,

        SERVO_MX         = 50,
            SERVO_MX12W  = 51,
            SERVO_MX28   = 52,
            SERVO_MX64   = 53,
            SERVO_MX106  = 54,

        SERVO_XL         = 60,
            SERVO_XL320  = 61,

        SERVO_X          = 70,
            SERVO_XM430_W210 = 71,
            SERVO_XM430_W350 = 72,

            SERVO_XH430_W210 = 81,
            SERVO_XH430_W350 = 82,
            SERVO_XH430_V210 = 83,
            SERVO_XH430_V350 = 84,

        // Dynamixel PRO servo devices:
        SERVO_PRO        = 170,
            SERVO_H54,
            SERVO_H42,
            SERVO_H30,
            SERVO_M54,
            SERVO_M42,
            SERVO_M30,
            SERVO_L54,
            SERVO_L42,
            SERVO_L30,

        // Dynamixel sensor devices:
        SENSOR_DYNAMIXEL     = 190,
            SENSOR_AXS1      = 191,
            SENSOR_IR_ARRAY  = 192,

    // HerkuleX servo devices:
    SERVO_HERKULEX       = 200,

        SERVO_DRS           = 210,
            SERVO_DRS_0101  = 211,
            SERVO_DRS_0201  = 212,
            SERVO_DRS_0401  = 213,
            SERVO_DRS_0402  = 214,
            SERVO_DRS_0601  = 215,
            SERVO_DRS_0602  = 216,
};

/*!
 * \brief The RegisterNames_e enum
 */
enum RegisterNames_e
{
    // Generic registers
    ////////////////////////////////////////////////////////////////////////////

    REG_MODEL_NUMBER = 0,
    REG_FIRMWARE_VERSION,
    REG_ID,
    REG_BAUD_RATE,
    REG_RETURN_DELAY_TIME,

    REG_MIN_POSITION, //!< "CW_ANGLE_LIMIT" for AX series; "MIN_POSITION_LIMIT" equivalent on PRO and HerkuleX ?
    REG_MAX_POSITION, //!< "CCW_ANGLE_LIMIT" for AX series; "MAX_POSITION_LIMIT" equivalent on PRO and HerkuleX ?

    REG_TEMPERATURE_LIMIT,
    REG_VOLTAGE_LOWEST_LIMIT,
    REG_VOLTAGE_HIGHEST_LIMIT,

    REG_MAX_TORQUE,
    REG_TORQUE_LIMIT,
    REG_STATUS_RETURN_LEVEL,    //!< Exact equivalent to "ACK_POLICY" on HerkuleX

    REG_LED,
    REG_LED_BLINKING,           //!< Only on HerkuleX
    REG_ALARM_LED,              //!< HerkuleX has ALARM_LED_POLICY, similar but not quite the same
    REG_ALARM_SHUTDOWN,         //!< HerkuleX has TORQUE_POLICY, similar but not quite the same

    REG_MULTI_TURN_OFFSET,      //!< Only on latest MX firmware
    REG_RESOLUTION_DIVIDER,     //!< Only on latest MX firmware

    REG_CW_COMPLIANCE_MARGIN ,  //!< "Compliance Margins" only on AX, DX and RX series
    REG_CCW_COMPLIANCE_MARGIN,
    REG_CW_COMPLIANCE_SLOPE,
    REG_CCW_COMPLIANCE_SLOPE,

    REG_P_GAIN,                 //!< "PID control" only on EX and MX, X and HerkuleX.
    REG_I_GAIN,
    REG_D_GAIN,

    REG_GOAL_POSITION,
    REG_GOAL_SPEED,
    REG_GOAL_VELOCITY,
    REG_GOAL_TORQUE,            //!< Only on MX-64, MX-106
    REG_GOAL_ACCELERATION,      //!< Only on MX, PRO

    REG_CURRENT_POSITION,
    REG_CURRENT_SPEED,
    REG_CURRENT_VELOCITY,
    REG_CURRENT_LOAD,
    REG_CURRENT_VOLTAGE,
    REG_CURRENT_TEMPERATURE,
    REG_CURRENT_CURRENT,        //!< Only on EX-106, MX-64 and MX-106

    REG_REGISTERED,             //!< "Registered Instruction" on Dynamixels
    REG_MOVING,
    REG_LOCK,
    REG_PUNCH,

    REG_TORQUE_ENABLE,          //!< Closest match on HerkuleX is "TORQUE_CONTROL"
    REG_CONTROL_MODE,           //!< Only on MX-64 and MX-106, X and HerkuleX devices.

    // Dynamixel specific registers
    ////////////////////////////////////////////////////////////////////////////

    REG_DRIVE_MODE,             //!< Only on EX, MX-106 and XH/XM
    REG_OPERATING_MODE,         //!< Only PRO
    REG_HW_ERROR_STATUS,        //!< Only on X, PRO // Similar to HerkuleX SERVO_STATUS_ERROR ???
    REG_MODEL_INFORMATION,      //!< Only PRO, XH/XM
    REG_HOMING_OFFSET,
    REG_MOVING_THRESHOLD,
    REG_ACCELERATION_LIMIT,
    REG_VELOCITY_LIMIT,
    REG_EXTERNAL_PORT_MODE_1,
    REG_EXTERNAL_PORT_MODE_2,
    REG_EXTERNAL_PORT_MODE_3,
    REG_EXTERNAL_PORT_MODE_4,
    REG_INDIRECT_ADDRESS_X,
    REG_SHUTDOWN,
    REG_LED_RED,
    REG_LED_GREEN,
    REG_LED_BLUE,
    REG_EXTERNAL_PORT_DATA_1,
    REG_EXTERNAL_PORT_DATA_2,
    REG_EXTERNAL_PORT_DATA_3,
    REG_EXTERNAL_PORT_DATA_4,
    REG_INDIRECT_DATA_X,
    REG_SHADOW_ID,
    REG_PROTOCOL_VERSION,
    REG_PWM_LIMIT,
    REG_CURRENT_LIMIT,
    REG_VELOCITY_I_GAIN,
    REG_VELOCITY_P_GAIN,
    REG_BUS_WATCHDOG,
    REG_GOAL_PWM,
    REG_GOAL_CURRENT,
    REG_PROFILE_ACCELERATION,
    REG_PROFILE_VELOCITY,
    REG_REALTIME_TICK,
    REG_MOVING_STATUS,
    REG_CURRENT_PWM,
    REG_VELOCITY_TRAJECTORY,
    REG_POSITION_TRAJECTORY,

    // HerkuleX specific registers
    ////////////////////////////////////////////////////////////////////////////

    REG_ABSOLUTE_POSITION,
    REG_CURRENT_CONTROL_MODE,
    REG_TICK,
    REG_CALIBRATED_POSITION,
    REG_DIFFERENTIAL_POSITION,
    REG_PWM,
    REG_ABSOLUTE_2nd_POSITION,
    REG_ABSOLUTE_GOAL_POSITION,
    REG_GOAL_TRAJECTORY,
    REG_ACCEL_RATIO,
    REG_MAX_ACCEL_TIME,
    REG_DEAD_ZONE,
    REG_SATURATOR_OFFSET,
    REG_SATURATOR_SLOPE,
    REG_PWM_OFFSET,
    REG_PWM_MIN,
    REG_PWM_MAX,
    REG_PWM_OVERLOAD_THRESHOLD,
    REG_POS_FEED_FRW_1st_GAIN,
    REG_POS_FEED_FRW_2nd_GAIN,
    REG_VELOCITY_KP,
    REG_VELOCITY_KI,
    REG_ADC_FAULT_CHECK_PRD,
    REG_PKT_GARBAGE_CHECK_PRD,
    REG_STOP_DETECTION_PRD,
    REG_OVERLOAD_DETECTION_PRD,
    REG_STOP_THRESHOLD,
    REG_INPOSITION_MARGIN,
    REG_CALIBRATION_DIFFERENCE,
    REG_STATUS_ERROR,
    REG_STATUS_DETAIL,
    REG_AUX_1,

    // SENSORS specific registers
    ////////////////////////////////////////////////////////////////////////////

    REG_IR_THRESHOLD_1,
    REG_IR_THRESHOLD_2,
    REG_IR_THRESHOLD_3,
    REG_IR_THRESHOLD_4,
    REG_IR_THRESHOLD_5,
    REG_IR_THRESHOLD_6,
    REG_IR_THRESHOLD_7,
    REG_IR_DATA_1,
    REG_IR_DATA_2,
    REG_IR_DATA_3,
    REG_IR_DATA_4,
    REG_IR_DATA_5,
    REG_IR_DATA_6,
    REG_IR_DATA_7,
    REG_IR_AUTO_THRESHOLD,

    REG_IR_DATA_LEFT,
    REG_IR_DATA_CENTER,
    REG_IR_DATA_RIGHT,
    REG_LIGHT_DATA_LEFT,
    REG_LIGHT_DATA_CENTER,
    REG_LIGHT_DATA_RIGHT,
    REG_IR_OBSTACLE_DETECTED,
    REG_LIGHT_DETECTED,
    REG_SOUND_DATA,
    REG_SOUND_DATA_MAX_HOLD,
    REG_SOUND_DETECTED_COUNT,
    REG_SOUND_DETECTED_TIME,
    REG_BUZZER_DATA_0,
    REG_BUZZER_DATA_1,
    REG_IR_REMOCON_ARRIVED,
    REG_REMOCON_RX_DATA_0,
    REG_REMOCON_RX_DATA_1,
    REG_REMOCON_TX_DATA_0,
    REG_REMOCON_TX_DATA_1,
    REG_IR_DETECT_COMPARE,
    REG_LIGHT_DETECT_COMPARE
};

/* ************************************************************************** */

/*!
 * \brief Assemble low and high bytes to make a short (16b) word.
 * \param lowbyte: Value of the low byte.
 * \param highbyte: Value of the high byte.
 * \return Short word made from the concatenation of low and high bytes.
 */
inline int make_short_word(const int lowbyte, const int highbyte)
{
    return ( ((highbyte & 0x000000FF) << 8) + (lowbyte & 0x000000FF) );
}

/*!
 * \brief Assemble bytes to make a regular (32b) word.
 * \param byte1: Value of the first byte.
 * \param byte2: Value of the second byte.
 * \param byte3: Value of the third byte.
 * \param byte4: Value of the fourth byte.
 * \return Word made from the concatenation of four byteq.
 */
inline int make_word(const int byte1, const int byte2, const int byte3, const int byte4)
{
    return ( ((byte4 & 0x000000FF) << 24) + ((byte3 & 0x000000FF) << 16) + ((byte2 & 0x000000FF) << 8) + (byte1 & 0x000000FF) );
}
/*!
 * \brief Assemble bytes to make a regular (32b) word.
 * \param short_low: Value of the first two bytes.
 * \param short_high: Value of the last two bytes.
 * \return Word made from the concatenation of four bytes.
 */
inline int make_word(const int short_low, const int short_high)
{
    return ( ((short_high & 0x0000FFFF) << 16) + (short_low & 0x0000FFFF) );
}

/*!
 * \brief Get the first (or 'low') byte from a regular 32b word.
 * \param word: The word from which to extract the low byte.
 * \return The value of the low byte as unsigned char.
 */
inline unsigned char get_lowbyte(const int word)
{
    return static_cast<unsigned char>(word & 0x000000FF);
}

/*!
 * \brief Get the second (or 'high') byte from a regular 32b word.
 * \param word: The word from which to extract the high byte.
 * \return The value of the high byte as unsigned char.
 */
inline unsigned char get_highbyte(const int word)
{
    return static_cast<unsigned char>((word & 0x0000FF00) >> 8);
}

/* ************************************************************************** */

#include <string>

/*!
 * \brief Get register description.
 * \param reg_name: Register name from '::RegisterNames_e' enum.
 * \return A string containing a textual description of a register.
 */
std::string getRegisterDescriptionTxt(const int reg_name);

/*!
 * \brief Get register name.
 * \param reg_name: Register name from '::RegisterNames_e' enum.
 * \return A string containing the name of a register.
 */
std::string getRegisterNameTxt(const int reg_name);

/** @}*/

/* ************************************************************************** */
#endif // UTILS_H
