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
 * \file tabSerial.cpp
 * \date 17/04/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "tabSerial.h"
#include "ui_tabSerial.h"

// SmartServoFramework
#include <SmartServoFramework/ManagedAPI.h>

tabSerial::tabSerial(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::tabSerial)
{
    ui->setupUi(this);
}

tabSerial::~tabSerial()
{
    delete ui;
}

void tabSerial::setInfos(int serial_state, std::string serial_path, int serial_baudrate,
                         int device_protocol, int device_connected)
{
    ui->label_title->setText(QString::fromStdString(serial_path));

    QString baudrate_qstr;
    if (serial_baudrate  >= 1000000)
        baudrate_qstr = QString::number(serial_baudrate / 1000000) + " Mbps";
    else if (serial_baudrate  > 1000)
        baudrate_qstr = QString::number(serial_baudrate / 1000.0, 'f', 1) + " Kbps";
    else
        baudrate_qstr = QString::number(serial_baudrate) + " bps";
    ui->label_speed_current->setText(baudrate_qstr);

    if (device_protocol == PROTOCOL_DXLv1)
        ui->label_protocol_current->setText("Dynamixel v1");
    else if (device_protocol == PROTOCOL_DXLv2)
        ui->label_protocol_current->setText("Dynamixel v2");
    else if (device_protocol == PROTOCOL_HKX)
        ui->label_protocol_current->setText("HerkuleX");

    ui->label_device_count->setText(QString::number(device_connected));

    QPixmap serialPortIcon;
    if (serial_state == 1)
    {
        if (device_connected > 0)
            serialPortIcon.load(":/icons/icons/network-transmit-receive.svg");
        else
            serialPortIcon.load(":/icons/icons/network-idle.svg");
    }
    else if (serial_state == 0)
        serialPortIcon.load(":/icons/icons/network-offline.svg");
    else if (serial_state == -1)
        serialPortIcon.load(":/icons/icons/emblem-readonly.svg");
    else
        serialPortIcon.load(":/icons/icons/network-error.svg");
    ui->label_icon->setPixmap(serialPortIcon);

    // TODO
    ui->label_7->hide();
    ui->label_speed_saved->hide();
    ui->label_speed_saved->setText("None");
    ui->label_8->hide();
    ui->label_protocol_saved->hide();
    ui->label_protocol_saved->setText("None");
}
