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
 * \file widgetSerialError.cpp
 * \date 22/04/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "src/widgetSerialError.h"
#include "ui_widgetSerialError.h"

// SmartServoFramework
#include "../../src/DynamixelController.h"
#include "../../src/HerkuleXController.h"
#include "../../src/DynamixelTools.h"
#include "../../src/HerkuleXTools.h"
#include "../../src/Servo.h"

widgetSerialError::widgetSerialError(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::widgetSerialError)
{
    ui->setupUi(this);

    QObject::connect(ui->pushButton_updateStatus, SIGNAL(clicked()), this, SIGNAL(updateButton()));
    QObject::connect(ui->pushButton_clearStatus, SIGNAL(clicked()), this, SIGNAL(clearButton()));
}

widgetSerialError::~widgetSerialError()
{
    delete ui;
}

void widgetSerialError::updateVisibility(const int servoSerie)
{
    if (servoSerie >= SERVO_HERKULEX)
    {
        ui->frame_err_dxl1->hide();
        ui->frame_err_dxl2->hide();
        ui->frame_err_hkx->show();
    }
    else
    {
        ui->frame_err_hkx->hide();

        if (servoSerie >= SERVO_XL)
        {
            ui->frame_err_dxl1->hide();
            ui->frame_err_dxl2->show();
        }
        else
        {
            ui->frame_err_dxl1->show();
            ui->frame_err_dxl2->hide();
        }
    }
}

void widgetSerialError::handleErrors(ControllerAPI *ctrl, Servo *servo,
                                     const int servoSerie, const int servoModel)
{
    QString css_comm_ok("color: white;\nborder: 1px solid rgb(85, 170, 0);\nbackground: rgba(85, 200, 0, 128);");
    QString css_error("border: 1px solid rgb(255, 53, 3);\nbackground: rgba(255, 170, 0, 128);\ncolor: white;");
    QString css_status("border-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(255, 248, 191, 128);\ncolor: rgb(246, 130, 9);");
    QString css_ok_middle("border-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");
    QString css_ok_left("border-left: 1px solid rgb(10, 100, 255);\nborder-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");
    QString css_ok_right("border-right: 1px solid rgb(10, 100, 255);\nborder-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");

    if (ctrl)
    {
        if (ctrl->getErrorCount() > 0)
        {
            ui->serialLinkErrors_label2->setStyleSheet(css_error);
        }
        else
        {
            ui->serialLinkErrors_label2->setStyleSheet(css_comm_ok);
        }
    }

    if (servo)
    {
        int srv_error = servo->getError();

        if (servoSerie >= SERVO_HERKULEX)
        {
            int srv_status = servo->getStatus();

            if (srv_error & ERRBIT_VOLTAGE)
                { ui->label_err_vin_2->setStyleSheet(css_error); }
            else
                { ui->label_err_vin_2->setStyleSheet(css_ok_left); }

            if (srv_error & ERRBIT_ALLOWED_POT)
                { ui->label_err_angle_2->setStyleSheet(css_error); }
            else
                { ui->label_err_angle_2->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_OVERHEAT)
                { ui->label_err_heat_2->setStyleSheet(css_error); }
            else
                { ui->label_err_heat_2->setStyleSheet(css_ok_middle); }

            if (srv_status & STATBIT_MOVING)
                { ui->label_stat_moving->setStyleSheet(css_status); }
            else
                { ui->label_stat_moving->setStyleSheet(css_ok_left); }

            if (srv_status & STATBIT_INPOSITION)
                { ui->label_stat_inpos->setStyleSheet(css_status); }
            else
                { ui->label_stat_inpos->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_INVALID_PKT)
            {
                ui->label_err_packet->setStyleSheet(css_status);

                if (srv_status & STATBIT_CHECKSUM_FLAG)
                    { ui->label_stat_crc->setStyleSheet(css_error); }
                else
                    { ui->label_stat_crc->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_UNKWOWN_CMD)
                    { ui->label_stat_cmd->setStyleSheet(css_error); }
                else
                    { ui->label_stat_cmd->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_RANGE)
                    { ui->label_stat_addr->setStyleSheet(css_error); }
                else
                    { ui->label_stat_addr->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_GARBAGE)
                    { ui->label_stat_gbg->setStyleSheet(css_error); }
                else
                    { ui->label_stat_gbg->setStyleSheet(css_ok_middle); }
            }
            else
            { ui->label_err_packet->setStyleSheet(css_ok_middle); }

            if (srv_status & STATBIT_TORQUE_ON)
                { ui->label_stat_ton->setStyleSheet(css_status); }
            else
                { ui->label_stat_ton->setStyleSheet(css_ok_right); }

            if (srv_error & ERRBIT_OVERLOAD)
                { ui->label_err_overload_2->setStyleSheet(css_error); }
            else
                { ui->label_err_overload_2->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_DRIVER_FAULT)
                { ui->label_err_driver->setStyleSheet(css_error); }
            else
                { ui->label_err_driver->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_EEP_REG_DIST)
                { ui->label_err_eep->setStyleSheet(css_error); }
            else
                { ui->label_err_eep->setStyleSheet(css_ok_right); }
        }
        else
        {
            if (servoSerie >= SERVO_XL)
            {
                if (srv_error == ERRBIT2_RESULT)
                    { ui->label_err_res->setStyleSheet(css_error); }
                else
                    { ui->label_err_res->setStyleSheet(css_ok_left); }

                if (srv_error == ERRBIT2_INSTRUCTION)
                    { ui->label_err_cmd_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_cmd_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_CHECKSUM)
                    { ui->label_err_crc_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_crc_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_RANGE)
                    { ui->label_err_range_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_range_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_LENGTH)
                    { ui->label_err_length->setStyleSheet(css_error); }
                else
                    { ui->label_err_length->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_LIMIT)
                    { ui->label_err_limit->setStyleSheet(css_error); }
                else
                    { ui->label_err_limit->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_ACCESS)
                    { ui->label_err_access->setStyleSheet(css_error); }
                else
                    { ui->label_err_access->setStyleSheet(css_ok_right); }
            }
            else
            {
                if (srv_error & 0x01)
                    { ui->label_err_vin->setStyleSheet(css_error); }
                else
                    { ui->label_err_vin->setStyleSheet(css_ok_right); }

                if (srv_error & 0x02)
                    { ui->label_err_angle->setStyleSheet(css_error); }
                else
                    { ui->label_err_angle->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x04)
                    { ui->label_err_heat->setStyleSheet(css_error); }
                else
                    { ui->label_err_heat->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x08)
                    { ui->label_err_range->setStyleSheet(css_error); }
                else
                    { ui->label_err_range->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x10)
                    { ui->label_err_crc->setStyleSheet(css_error); }
                else
                    { ui->label_err_crc->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x20)
                    { ui->label_err_overload->setStyleSheet(css_error); }
                else
                    { ui->label_err_overload->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x40)
                    { ui->label_err_cmd->setStyleSheet(css_error); }
                else
                    { ui->label_err_cmd->setStyleSheet(css_ok_left); }
            }
        }
    }
}
