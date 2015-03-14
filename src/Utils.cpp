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
 * \file Utils.cpp
 * \date 08/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "Utils.h"

std::string getRegisterNameTxt(const int reg_name)
{
    std::string name;

    switch (reg_name)
    {
    case REG_MODEL_NUMBER:
        name = "MODEL_NUMBER";
        break;
    case REG_FIRMWARE_VERSION:
        name = "FIRMWARE_VERSION";
        break;
    case REG_ID:
        name = "SERVO ID";
        break;
    case REG_BAUD_RATE:
        name = "BAUD_RATE";
        break;
    case REG_RETURN_DELAY_TIME:
        name = "RETURN_DELAY_TIME";
        break;
    case REG_MIN_POSITION:
        name = "MIN_POSITION";
        break;
    case REG_MAX_POSITION:
        name = "MAX_POSITION";
        break;
    case REG_TEMPERATURE_LIMIT:
        name = "TEMPERATURE_LIMIT";
        break;
    case REG_VOLTAGE_LOWEST_LIMIT:
        name = "VOLTAGE_LOWEST_LIMIT";
        break;
    case REG_VOLTAGE_HIGHEST_LIMIT:
        name = "VOLTAGE_HIGHEST_LIMIT";
        break;
    case REG_MAX_TORQUE:
        name = "MAX_TORQUE";
        break;
    case REG_TORQUE_LIMIT:
        name = "TORQUE_LIMIT";
        break;
    case REG_STATUS_RETURN_LEVEL:
        name = "STATUS_RETURN_LEVEL";
        break;
    case REG_LED:
        name = "LED";
        break;
    case REG_LED_BLINKING:
        name = "LED_BLINKING";
        break;
    case REG_ALARM_LED:
        name = "ALARM_LED";
        break;
    case REG_ALARM_SHUTDOWN:
        name = "ALARM_SHUTDOWN";
        break;
    case REG_MULTI_TURN_OFFSET:
        name = "MULTI_TURN_OFFSET";
        break;
    case REG_RESOLUTION_DIVIDER:
        name = "RESOLUTION_DIVIDER";
        break;
    case REG_CW_COMPLIANCE_MARGIN :
        name = "CW_COMPLIANCE_MARGIN";
        break;
    case REG_CCW_COMPLIANCE_MARGIN:
        name = "CCW_COMPLIANCE_MARGIN";
        break;
    case REG_CW_COMPLIANCE_SLOPE:
        name = "CW_COMPLIANCE_SLOPE";
        break;
    case REG_CCW_COMPLIANCE_SLOPE:
        name = "CCW_COMPLIANCE_SLOPE";
        break;
    case REG_P_GAIN:
        name = "P_GAIN";
        break;
    case REG_I_GAIN:
        name = "I_GAIN";
        break;
    case REG_D_GAIN:
        name = "D_GAIN";
        break;
    case REG_GOAL_POSITION:
        name = "GOAL_POSITION";
        break;
    case REG_GOAL_SPEED:
        name = "GOAL_SPEED";
        break;
    case REG_GOAL_VELOCITY:
        name = "GOAL_VELOCITY";
        break;
    case REG_GOAL_TORQUE:
        name = "GOAL_TORQUE";
        break;
    case REG_GOAL_ACCELERATION:
        name = "GOAL_ACCELERATION";
        break;
    case REG_CURRENT_POSITION:
        name = "CURRENT_POSITION";
        break;
    case REG_CURRENT_SPEED:
        name = "CURRENT_SPEED";
        break;
    case REG_CURRENT_VELOCITY:
        name = "CURRENT_VELOCITY";
        break;
    case REG_CURRENT_LOAD:
        name = "CURRENT_LOAD";
        break;
    case REG_CURRENT_VOLTAGE:
        name = "CURRENT_VOLTAGE";
        break;
    case REG_CURRENT_TEMPERATURE:
        name = "CURRENT_TEMPERATURE";
        break;
    case REG_CURRENT_CURRENT:
        name = "CURRENT_CURRENT";
        break;
    case REG_REGISTERED:
        name = "REGISTERED";
        break;
    case REG_MOVING:
        name = "MOVING";
        break;
    case REG_LOCK:
        name = "LOCK";
        break;
    case REG_PUNCH:
        name = "PUNCH";
        break;
    case REG_TORQUE_ENABLE:
        name = "TORQUE_ENABLE";
        break;
    case REG_DRIVE_MODE:
        name = "DRIVE_MODE";
        break;
    case REG_CONTROL_MODE:
        name = "CONTROL_MODE";
        break;
    case REG_OPERATING_MODE:
        name = "OPERATING_MODE";
        break;
    case REG_HW_ERROR_STATUS:
        name = "HW_ERROR_STATUS";
        break;
    case REG_MODEL_INFORMATION:
        name = "MODEL_INFORMATION";
        break;

    case REG_HOMING_OFFSET:
        name = "HOMING_OFFSET";
        break;
    case REG_MOVING_THRESHOLD:
        name = "MOVING_THRESHOLD";
        break;
    case REG_ACCELERATION_LIMIT:
        name = "ACCELERATION_LIMIT";
        break;
    case REG_VELOCITY_LIMIT:
        name = "VELOCITY_LIMIT";
        break;

    case REG_ABSOLUTE_POSITION:
        name = "ABSOLUTE_POSITION";
        break;
    case REG_CURRENT_CONTROL_MODE:
        name = "CURRENT_CONTROL_MODE";
        break;
    case REG_TICK:
        name = "TICK";
        break;
    case REG_CALIBRATED_POSITION:
        name = "CALIBRATED_POSITION";
        break;
    case REG_DIFFERENTIAL_POSITION:
        name = "DIFFERENTIAL_POSITION";
        break;
    case REG_PWM:
        name = "PWM";
        break;
    case REG_ABSOLUTE_GOAL_POSITION:
        name = "ABSOLUTE_GOAL_POSITION";
        break;
    case REG_GOAL_TRAJECTORY:
        name = "GOAL_TRAJECTORY";
        break;
    case REG_ACCEL_RATIO:
        name = "ACCEL_RATIO";
        break;
    case REG_MAX_ACCEL_TIME:
        name = "MAX_ACCEL_TIME";
        break;
    case REG_DEAD_ZONE:
        name = "DEAD_ZONE";
        break;
    case REG_SATURATOR_OFFSET:
        name = "SATURATOR_OFFSET";
        break;
    case REG_SATURATOR_SLOPE:
        name = "SATURATOR_SLOPE";
        break;
    case REG_PWM_OFFSET:
        name = "PWM_OFFSET";
        break;
    case REG_PWM_MIN:
        name = "PWM_MIN";
        break;
    case REG_PWM_MAX:
        name = "PWM_MAX";
        break;
    case REG_PWM_OVERLOAD_THRESHOLD:
        name = "PWM_OVERLOAD_THRESHOLD";
        break;
    case REG_POS_FEED_FRW_1st_GAIN:
        name = "POS_FEED_FRW_1stGAIN";
        break;
    case REG_POS_FEED_FRW_2nd_GAIN:
        name = "POS_FEED_FRW_2ndGAIN";
        break;
    case REG_VELOCITY_KP:
        name = "REG_VELOCITY_KP";
        break;
    case REG_VELOCITY_KI:
        name = "REG_VELOCITY_KI";
        break;
    case REG_ADC_FAULT_CHECK_PRD:
        name = "ADC_FAULT_CHECK_PRD";
        break;
    case REG_PKT_GARBAGE_CHECK_PRD:
        name = "PKT_GARBAGE_CHECK_PRD";
        break;
    case REG_STOP_DETECTION_PRD:
        name = "STOP_DETECTION_PRD";
        break;
    case REG_OVERLOAD_DETECTION_PRD:
        name = "OVERLOAD_DETECTION_PRD";
        break;
    case REG_STOP_THRESHOLD:
        name = "STOP_THRESHOLD";
        break;
    case REG_INPOSITION_MARGIN:
        name = "INPOSITION_MARGIN";
        break;
    case REG_CALIBRATION_DIFFERENCE:
        name = "CALIBRATION_DIFFERENCE";
        break;
    case REG_STATUS_ERROR:
        name = "STATUS_ERROR";
        break;
    case REG_STATUS_DETAIL:
        name = "STATUS_DETAIL";
        break;
    case REG_AUX_1:
        name = "REG_AUX_1";
        break;

    case REG_IR_THRESHOLD_1:
        name = "SENSOR_IR_THRESHOLD_1";
        break;
    case REG_IR_THRESHOLD_2:
        name = "SENSOR_IR_THRESHOLD_2";
        break;
    case REG_IR_THRESHOLD_3:
        name = "SENSOR_IR_THRESHOLD_3";
        break;
    case REG_IR_THRESHOLD_4:
        name = "SENSOR_IR_THRESHOLD_4";
        break;
    case REG_IR_THRESHOLD_5:
        name = "SENSOR_IR_THRESHOLD_5";
        break;
    case REG_IR_THRESHOLD_6:
        name = "SENSOR_IR_THRESHOLD_6";
        break;
    case REG_IR_THRESHOLD_7:
        name = "SENSOR_IR_THRESHOLD_7";
        break;
    case REG_IR_DATA_1:
        name = "SENSOR_IR_DATA_1";
        break;
    case REG_IR_DATA_2:
        name = "SENSOR_IR_DATA_2";
        break;
    case REG_IR_DATA_3:
        name = "SENSOR_IR_DATA_3";
        break;
    case REG_IR_DATA_4:
        name = "SENSOR_IR_DATA_4";
        break;
    case REG_IR_DATA_5:
        name = "SENSOR_IR_DATA_5";
        break;
    case REG_IR_DATA_6:
        name = "SENSOR_IR_DATA_6";
        break;
    case REG_IR_DATA_7:
        name = "SENSOR_IR_DATA_7";
        break;
    case REG_IR_AUTO_THRESHOLD:
        name = "SENSOR_IR_AUTO_THRESHOLD";
        break;

    default:
        name = "REGISTER NAME";
        break;
    }

    return name;
}

std::string getRegisterDescriptionTxt(const int reg_name)
{
    std::string desc;

    switch (reg_name)
    {
    case REG_MODEL_NUMBER:
        desc = "Model Number";
        break;
    case REG_FIRMWARE_VERSION:
        desc = "Firmware Version";
        break;
    case REG_ID:
        desc = "Servo ID";
        break;
    case REG_BAUD_RATE:
        desc = "Baud Rate";
        break;
    case REG_RETURN_DELAY_TIME:
        desc = "Return Delay Time";
        break;
    case REG_MIN_POSITION:
        desc = "Clockwise (CW) Angle Limit";
        break;
    case REG_MAX_POSITION:
        desc = "Counterclockwise (CCW) Angle Limit";
        break;
    case REG_TEMPERATURE_LIMIT:
        desc = "Internal Limit Temperature";
        break;
    case REG_VOLTAGE_LOWEST_LIMIT:
        desc = "Lowest Limit Voltage";
        break;
    case REG_VOLTAGE_HIGHEST_LIMIT:
        desc = "Upper Limit Voltage";
        break;
    case REG_MAX_TORQUE:
        desc = "Maximum Torque";
        break;
    case REG_TORQUE_LIMIT:
        desc = "Torque Limit";
        break;
    case REG_STATUS_RETURN_LEVEL:
        desc = "Status Return Level";
        break;
    case REG_LED:
        desc = "LED (on/off)";
        break;
    case REG_LED_BLINKING:
        desc = "LED Blink Period";
        break;
    case REG_ALARM_LED:
        desc = "LED for Alarm";
        break;
    case REG_ALARM_SHUTDOWN:
        desc = "Shutdown for Alarm";
        break;
    case REG_MULTI_TURN_OFFSET:
        desc = "Multi Turn Offset";
        break;
    case REG_RESOLUTION_DIVIDER:
        desc = "Resolution Divider";
        break;
    case REG_CW_COMPLIANCE_MARGIN :
        desc = "CW (clockwise) Compliance margin";
        break;
    case REG_CCW_COMPLIANCE_MARGIN:
        desc = "CCW (counter clockwise) Compliance margin";
        break;
    case REG_CW_COMPLIANCE_SLOPE:
        desc = "CW (clockwise) Compliance slope";
        break;
    case REG_CCW_COMPLIANCE_SLOPE:
        desc = "CCW (counter clockwise) Compliance slope";
        break;
    case REG_P_GAIN:
        desc = "Proportional Gain";
        break;
    case REG_I_GAIN:
        desc = "Integral Gain";
        break;
    case REG_D_GAIN:
        desc = "Derivative Gain";
        break;
    case REG_GOAL_POSITION:
        desc = "Goal Position";
        break;
    case REG_GOAL_SPEED:
        desc = "Goal Speed";
        break;
    case REG_GOAL_VELOCITY:
        desc = "Goal Velocity";
        break;
    case REG_GOAL_TORQUE:
        desc = "Goal Torque";
        break;
    case REG_GOAL_ACCELERATION:
        desc = "Goal Acceleration";
        break;
    case REG_CURRENT_POSITION:
        desc = "Current Position";
        break;
    case REG_CURRENT_SPEED:
        desc = "Current Speed";
        break;
    case REG_CURRENT_VELOCITY:
        desc = "Current Velocity";
        break;
    case REG_CURRENT_LOAD:
        desc = "Current Load";
        break;
    case REG_CURRENT_VOLTAGE:
        desc = "Current Voltage";
        break;
    case REG_CURRENT_TEMPERATURE:
        desc = "Current Temperature";
        break;
    case REG_CURRENT_CURRENT:
        desc = "Current Current";
        break;
    case REG_REGISTERED:
        desc = "Instruction Currently Registered";
        break;
    case REG_MOVING:
        desc = "Moving";
        break;
    case REG_LOCK:
        desc = "Locking EEPROM";
        break;
    case REG_PUNCH:
        desc = "Punch";
        break;
    case REG_TORQUE_ENABLE:
        desc = "Torque (on/off)";
        break;
    case REG_DRIVE_MODE:
        desc = "Drive mode";
        break;
    case REG_CONTROL_MODE:
        desc = "Control Mode";
        break;
    case REG_OPERATING_MODE:
        desc = "Operating Mode";
        break;
    case REG_HW_ERROR_STATUS:
        desc = "Hardware Error Status";
        break;
    case REG_MODEL_INFORMATION:
        desc = "Model Information";
        break;

    case REG_HOMING_OFFSET:
        desc = "Homing Offset";
        break;
    case REG_MOVING_THRESHOLD:
        desc = "Moving Threshold";
        break;
    case REG_ACCELERATION_LIMIT:
        desc = "Acceleration Limit";
        break;
    case REG_VELOCITY_LIMIT:
        desc = "Velocity Limit";
        break;

    case REG_ABSOLUTE_POSITION:
        desc = "Absolute Position";
        break;
    case REG_CURRENT_CONTROL_MODE:
        desc = "Current Control Mode";
        break;
    case REG_TICK:
        desc = "Tick";
        break;
    case REG_CALIBRATED_POSITION:
        desc = "Calibrated Position";
        break;
    case REG_DIFFERENTIAL_POSITION:
        desc = "Differential Position";
        break;
    case REG_PWM:
        desc = "PWM (Pulse Width Modulation)";
        break;
    case REG_ABSOLUTE_GOAL_POSITION:
        desc = "Absolute Goal Position";
        break;
    case REG_GOAL_TRAJECTORY:
        desc = "Goal Trajectory";
        break;
    case REG_ACCEL_RATIO:
        desc = "Acceleration Ratio";
        break;
    case REG_MAX_ACCEL_TIME:
        desc = "Maximum Acceleration Time";
        break;
    case REG_DEAD_ZONE:
        desc = "Dead Zone";
        break;
    case REG_SATURATOR_OFFSET:
        desc = "Saturator Offset";
        break;
    case REG_SATURATOR_SLOPE:
        desc = "Saturator Slope";
        break;
    case REG_PWM_OFFSET:
        desc = "PWM Offset";
        break;
    case REG_PWM_MIN:
        desc = "Minimum PWM";
        break;
    case REG_PWM_MAX:
        desc = "Maximum PWM";
        break;
    case REG_PWM_OVERLOAD_THRESHOLD:
        desc = "PWM Overload Threshold";
        break;
    case REG_POS_FEED_FRW_1st_GAIN:
        desc = "Position Feedforward 1st Gain";
        break;
    case REG_POS_FEED_FRW_2nd_GAIN:
        desc = "Position Feedforward 2nd Gain";
        break;
    case REG_VELOCITY_KP:
        desc = "Velocity controller P Gain";
        break;
    case REG_VELOCITY_KI:
        desc = "Velocity controller I Gain";
        break;
    case REG_ADC_FAULT_CHECK_PRD:
        desc = "ADC Fault Check Period";
        break;
    case REG_PKT_GARBAGE_CHECK_PRD:
        desc = "Packet Garbage Check Period";
        break;
    case REG_STOP_DETECTION_PRD:
        desc = "Stop Detection Period";
        break;
    case REG_OVERLOAD_DETECTION_PRD:
        desc = "Overload Detection Period";
        break;
    case REG_STOP_THRESHOLD:
        desc = "Stop Threshold";
        break;
    case REG_INPOSITION_MARGIN:
        desc = "In-Position Margin";
        break;
    case REG_CALIBRATION_DIFFERENCE:
        desc = "Calibration difference";
        break;
    case REG_STATUS_ERROR:
        desc = "Status Error";
        break;
    case REG_STATUS_DETAIL:
        desc = "Status Detail";
        break;
    case REG_AUX_1:
        desc = "REG_AUX_1";
        break;

    case REG_IR_THRESHOLD_1:
        desc = "SENSOR_IR_THRESHOLD_1";
        break;
    case REG_IR_THRESHOLD_2:
        desc = "SENSOR_IR_THRESHOLD_2";
        break;
    case REG_IR_THRESHOLD_3:
        desc = "SENSOR_IR_THRESHOLD_3";
        break;
    case REG_IR_THRESHOLD_4:
        desc = "SENSOR_IR_THRESHOLD_4";
        break;
    case REG_IR_THRESHOLD_5:
        desc = "SENSOR_IR_THRESHOLD_5";
        break;
    case REG_IR_THRESHOLD_6:
        desc = "SENSOR_IR_THRESHOLD_6";
        break;
    case REG_IR_THRESHOLD_7:
        desc = "SENSOR_IR_THRESHOLD_7";
        break;
    case REG_IR_DATA_1:
        desc = "SENSOR_IR_DATA_1";
        break;
    case REG_IR_DATA_2:
        desc = "SENSOR_IR_DATA_2";
        break;
    case REG_IR_DATA_3:
        desc = "SENSOR_IR_DATA_3";
        break;
    case REG_IR_DATA_4:
        desc = "SENSOR_IR_DATA_4";
        break;
    case REG_IR_DATA_5:
        desc = "SENSOR_IR_DATA_5";
        break;
    case REG_IR_DATA_6:
        desc = "SENSOR_IR_DATA_6";
        break;
    case REG_IR_DATA_7:
        desc = "SENSOR_IR_DATA_7";
        break;
    case REG_IR_AUTO_THRESHOLD:
        desc = "SENSOR_IR_AUTO_THRESHOLD";
        break;

    default:
        desc = "REGISTER DESCRIPTION";
        break;
    }

    return desc;
}
