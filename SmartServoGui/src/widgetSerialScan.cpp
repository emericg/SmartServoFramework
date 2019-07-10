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
#include <SmartServoFramework/ManagedAPI.h>

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
    loadSavedParameters();
}

widgetSerialScan::~widgetSerialScan()
{
    if (ui->portMode->currentIndex() == Manual) {
        m_settings.beginGroup("serialport");
        m_settings.beginGroup(ui->portName->text());
        m_settings.setValue("firstAddress", ui->rangeStart_spinBox->value());
        m_settings.setValue("lastAddress", ui->rangeStop_spinBox->value());
        m_settings.endGroup(); // end port
        m_settings.endGroup(); // end serialport
    }
    delete ui;
}

void widgetSerialScan::setSavedParameters(ServoProtocol protocol, int speed, int firstAddress, int lastAddress)
{
    saved_protocol = protocol;
    saved_speed = speed;
    ui->comboBox_protocol->setCurrentProcotol(protocol);
    ui->comboBox_baudrate->setBaudrate(saved_speed);

    if (ui->portMode->itemText(ui->portMode->count()-1) != tr("saved"))
        ui->portMode->addItem(tr("saved"));
    ui->portMode->setCurrentIndex(ui->portMode->count()-1);

    if (firstAddress <= lastAddress) {
        ui->rangeStart_spinBox->setValue(firstAddress);
        ui->rangeStop_spinBox->setValue(lastAddress);

        ui->rangeStart_spinBox->setMaximum(ui->rangeStop_spinBox->value());
        ui->rangeStop_spinBox->setMinimum(ui->rangeStart_spinBox->value());
    } else {
        // malformed config
        ui->rangeStart_spinBox->setValue(0);
        ui->rangeStart_spinBox->setMaximum(253);
        ui->rangeStop_spinBox->setValue(253);
        ui->rangeStop_spinBox->setMinimum(0);
    }
}

void widgetSerialScan::loadSavedParameters()
{
    m_settings.beginGroup("serialport");
    m_settings.beginGroup(ui->portName->text());
    ui->rangeStart_spinBox->setValue(m_settings.value("firstAddress", 0).toInt());
    ui->rangeStop_spinBox->setValue(m_settings.value("lastAddress", 253).toInt());
    ui->comboBox_protocol->setCurrentProcotol(static_cast<ServoProtocol>(m_settings.value("protocol", ServoProtocol::PROTOCOL_DXLv1).toInt()));
    ui->comboBox_baudrate->setBaudrate(m_settings.value("speed", 1000000).toInt());
    ui->portName->setChecked(m_settings.value("enabled", false).toBool());
    m_settings.endGroup(); // end port
    m_settings.endGroup(); // end serialport
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
    int speed = 0;
    if (index == -1)
        index = ui->portMode->currentIndex();

    switch (static_cast<PortMode>(index))
    {
    case Manual:
        speed = ui->comboBox_baudrate->currentBaudrate();
        break;
    case DXL_V1_1Mb:
    case DXL_V2_1Mb:
    case HKX_1Mb:
        speed = 1000000;
        break;
    case DXL_V1_115k:
    case DXL_V2_115k:
    case HKX_115k:
        speed = 115200;
        break;
    case DXL_V1_57p6k:
    case DXL_V2_57p6k:
    case HKX_57p6k:
        speed = 57600;
        break;
    case Saved:
        speed = saved_speed;
        break;
    default:
        break;
    }

    return speed;
}

ServoProtocol widgetSerialScan::getCurrentProtocol(int index)
{
    if (index == -1)
        index = ui->portMode->currentIndex();

    switch (index)
    {
    case Manual:
        return ui->comboBox_protocol->currentProtocol();
    case DXL_V1_1Mb:
    case DXL_V1_115k:
    case DXL_V1_57p6k:
        return PROTOCOL_DXLv1;
    case DXL_V2_1Mb:
    case DXL_V2_115k:
    case DXL_V2_57p6k:
        return PROTOCOL_DXLv2;
    case HKX_1Mb:
    case HKX_115k:
    case HKX_57p6k:
        return PROTOCOL_HKX;
    case Saved:
        return saved_protocol;
    }
    return  PROTOCOL_UNKNOWN;
}

int widgetSerialScan::getCurrentDeviceClass(int index)
{
    if (index == -1)
        index = ui->portMode->currentIndex();

    switch (index)
    {
    case Manual:
    case Saved:
        switch (ui->comboBox_protocol->currentProtocol()) {
        case PROTOCOL_UNKNOWN:
            return SERVO_UNKNOWN;
        case PROTOCOL_DXLv1:
            return SERVO_MX;
        case PROTOCOL_DXLv2:
            return SERVO_XL;
        case PROTOCOL_HKX:
            return SERVO_DRS;
        }
        break;
    case DXL_V1_1Mb:
    case DXL_V1_115k:
    case DXL_V1_57p6k:
        return SERVO_MX;
    case DXL_V2_1Mb:
    case DXL_V2_115k:
    case DXL_V2_57p6k:
        return SERVO_XL;
    case HKX_1Mb:
    case HKX_115k:
    case HKX_57p6k:
        return SERVO_DRS;
    }
    return SERVO_UNKNOWN;
}

int widgetSerialScan::getMinRange()
{
    if (ui->portMode->currentIndex() == Manual
            || ui->portMode->currentIndex() == Saved)
        return ui->rangeStart_spinBox->value();
    return 0;
}

int widgetSerialScan::getMaxRange()
{
    if (ui->portMode->currentIndex() == Manual
            || ui->portMode->currentIndex() == Saved)
        return ui->rangeStop_spinBox->value();
    return 253;
}

void widgetSerialScan::on_portMode_currentIndexChanged(int index)
{
    if (index == Manual)
    {
        // manual settings
        ui->wProtocol->show();
        ui->wSpeed->show();
        ui->wRange->show();
    }
    else
    {
        if (index == Saved)
        {
            // saved settings
            ui->portMode->setCurrentIndex(ui->portMode->count()-1);
            ui->comboBox_protocol->setCurrentProcotol(saved_protocol);
            ui->comboBox_baudrate->setBaudrate(saved_speed);
        }

        ui->wProtocol->hide();
        ui->wSpeed->hide();
        ui->wRange->hide();
    }
}

void widgetSerialScan::on_portScanButton_clicked()
{
    emit scanButton(ui->portName->text());
}

void widgetSerialScan::on_rangeStop_spinBox_valueChanged(int arg1)
{
    ui->rangeStart_spinBox->setMaximum(arg1);
}

void widgetSerialScan::on_rangeStart_spinBox_valueChanged(int arg1)
{
    ui->rangeStop_spinBox->setMinimum(arg1);
}
