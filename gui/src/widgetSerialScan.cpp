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
 * \file widgetSerialScan.cpp
 * \date 03/05/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "src/widgetSerialScan.h"
#include "ui_widgetSerialScan.h"

// SmartServoFramework
#include "../../src/Dynamixel.h"
#include "../../src/HerkuleX.h"

#include <QDebug>

widgetSerialScan::widgetSerialScan(QString &port, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::widgetSerialScan)
{
    ui->setupUi(this);

    ui->portName->setText(port);

    ui->wProtocol->hide();
    ui->wSpeed->hide();
    ui->wRange->hide();
}

widgetSerialScan::~widgetSerialScan()
{
    delete ui;
}

std::string widgetSerialScan::getDeviceName()
{
    return ui->portName->text().toStdString();
}

bool widgetSerialScan::isSelected()
{
    return ui->portName->isChecked();
}

int widgetSerialScan::getCurrentIndex()
{
    return ui->portMode->currentIndex();
}

int widgetSerialScan::getCurrentSpeed(int index)
{
    if (index == -1)
        index = ui->portMode->currentIndex();
    int speed = 0;

    switch (index)
    {
    case 1:
        speed = ui->spinBox_speed->value();
        break;

    case 2:
    case 5:
    case 8:
        speed = 1000000;
        break;

    case 3:
    case 6:
    case 9:
        speed = 115200;
        break;

    case 4:
    case 7:
    case 10:
        speed = 57600;
        break;

    default:
        break;
    }

    return speed;
}

int widgetSerialScan::getCurrentProtocol(int index)
{
    if (index == -1)
        index = ui->portMode->currentIndex();
    int protocol = 0;

    switch (index)
    {
    case 1:
        protocol = ui->comboBox_protocol->currentIndex() + 1;
        break;

    case 2:
    case 3:
    case 4:
        protocol = PROTOCOL_DXLv1;
        break;
    case 5:
    case 6:
    case 7:
        protocol = PROTOCOL_DXLv2;
        break;
    case 8:
    case 9:
    case 10:
        protocol = PROTOCOL_HKX;
        break;

    default:
        break;
    }

    return protocol;
}

int widgetSerialScan::getCurrentDeviceClass(int index)
{
    if (index == -1)
        index = ui->portMode->currentIndex();
    int deviceClass = 0;

    switch (index)
    {
    case 1:
        if (ui->comboBox_protocol->currentIndex() == 0)
            deviceClass = SERVO_MX;
        else if (ui->comboBox_protocol->currentIndex() == 1)
            deviceClass = SERVO_XL;
        else if (ui->comboBox_protocol->currentIndex() == 2)
            deviceClass = SERVO_DRS;
        break;

    case 2:
    case 3:
    case 4:
        deviceClass = SERVO_MX;
        break;
    case 5:
    case 6:
    case 7:
        deviceClass = SERVO_XL;
        break;
    case 8:
    case 9:
    case 10:
        deviceClass = SERVO_DRS;
        break;

    default:
        break;
    }

    return deviceClass;
}

int widgetSerialScan::getMinRange()
{
    if (ui->portMode->currentIndex() == 1)
        return ui->rangeStart_spinBox->value();

    return 0;
}

int widgetSerialScan::getMaxRange()
{
    if (ui->portMode->currentIndex() == 1)
        return ui->rangeStop_spinBox->value();

    return 253;
}

void widgetSerialScan::on_portMode_currentIndexChanged(int index)
{
    if (index == 1)
    {
        // manual settings
        ui->wProtocol->show();
        ui->wSpeed->show();
        ui->wRange->show();
    }
    else
    {
        ui->wProtocol->hide();
        ui->wSpeed->hide();
        ui->wRange->hide();
    }
}

void widgetSerialScan::on_portScanButton_clicked()
{
    emit scanButton(ui->portName->text());
}
