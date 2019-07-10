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
 * \file settings.cpp
 * \date 05/11/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "settings.h"
#include "ui_settings.h"

#include "baudratecombobox.h"
#include "protocolcombobox.h"

#include <QtSerialPort/QSerialPortInfo>

Settings::Settings(QWidget *parent):
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
    loadSettings();
}

Settings::~Settings()
{
    delete ui;
}

void Settings::addPortWidgets(const portConfig portCfg, int row)
{
    QCheckBox *enabledCheckBox = new QCheckBox(this);
    enabledCheckBox->setChecked(portCfg.enabled);
    ui->gridLayoutPortsPort->addWidget(enabledCheckBox, row, Col_Enabled);

    QLabel *portNameLabel = new QLabel(this);
    portNameLabel->setText(portCfg.path);
    ui->gridLayoutPortsPort->addWidget(portNameLabel, row, Col_PortName);

    ProtocolComboBox *protocolCombo = new ProtocolComboBox(this);
    protocolCombo->setCurrentProcotol(portCfg.protocol);
    ui->gridLayoutPortsPort->addWidget(protocolCombo, row, Col_Protocol);

    BaudrateComboBox *baudcomboBox = new BaudrateComboBox(this);
    baudcomboBox->setBaudrate(portCfg.speed);
    ui->gridLayoutPortsPort->addWidget(baudcomboBox, row, Col_Baud);

    QSpinBox *firstAddressSpinBox = new QSpinBox(this);
    firstAddressSpinBox->setMinimum(0);
    firstAddressSpinBox->setMaximum(portCfg.lastAddress);
    firstAddressSpinBox->setValue(portCfg.firstAddress);
    ui->gridLayoutPortsPort->addWidget(firstAddressSpinBox, row, Col_FirstAddress);

    QSpinBox *lastAddressSpinBox = new QSpinBox(this);
    lastAddressSpinBox->setMinimum(portCfg.firstAddress);
    lastAddressSpinBox->setMaximum(253);
    lastAddressSpinBox->setValue(portCfg.lastAddress);
    ui->gridLayoutPortsPort->addWidget(lastAddressSpinBox, row, Col_LastAddress);

    connect(firstAddressSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            lastAddressSpinBox, [=](int value){lastAddressSpinBox->setMinimum(value);});
    connect(lastAddressSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            firstAddressSpinBox, [=](int value){firstAddressSpinBox->setMaximum(value);});
}

void Settings::loadSerialPorts()
{
    int row = 1;
    m_settings.beginGroup("serialport");
    for (auto portInfo : QSerialPortInfo::availablePorts()) {
        m_settings.beginGroup(portInfo.systemLocation());

        struct portConfig config;
        config.path = portInfo.systemLocation();
        config.enabled = m_settings.value("enabled", config.enabled).toBool();
        config.speed = m_settings.value("speed", config.speed).toInt();
        config.protocol = static_cast<ServoProtocol>(m_settings.value("protocol", config.protocol).toInt());
        config.firstAddress = m_settings.value("firstAddress", config.firstAddress).toInt();
        config.lastAddress = m_settings.value("lastAddress", config.lastAddress).toInt();
        m_settings.endGroup();
        serial_ports.push_back(config);
        addPortWidgets(config, row);
        row++;
    }
    m_settings.endGroup();
}

void Settings::loadSettings()
{
    m_settings.beginGroup("gui");
    ui_pause = m_settings.value("pause", false).toBool();
    m_settings.endGroup();

    m_settings.beginGroup("controller");
    ctrl_autoscan = m_settings.value("ctrl_autoscan", true).toBool();
    ctrl_locks = m_settings.value("ctrl_locks", true).toBool();
    ctrl_freq = m_settings.value("ctrl_freq", 10).toInt();
    m_settings.endGroup();

    ui->checkBox_pause->setChecked(ui_pause);
    ui->checkBox_autoscan->setChecked(ctrl_autoscan);
    ui->checkBox_locks->setChecked(ctrl_locks);
    ui->spinBox_freq->setValue(ctrl_freq);

    loadSerialPorts();
}

void Settings::on_pushButton_save_clicked()
{
    ui_pause = ui->checkBox_pause->isChecked();
    ctrl_autoscan = ui->checkBox_autoscan->isChecked();
    ctrl_locks = ui->checkBox_locks->isChecked();
    ctrl_freq = ui->spinBox_freq->value();

    m_settings.beginGroup("gui");
    m_settings.setValue("pause", ui_pause);
    m_settings.endGroup();

    m_settings.beginGroup("controller");
    m_settings.setValue("ctrl_autoscan", ctrl_autoscan);
    m_settings.setValue("ctrl_locks", ctrl_locks);
    m_settings.setValue("ctrl_freq", ctrl_freq);
    m_settings.endGroup();

    m_settings.beginGroup("serialport");
    for (int row = 1; row<(serial_ports.count() + 1); row++) {
        QString portName = static_cast<QLabel*>(
                    ui->gridLayoutPortsPort->itemAtPosition(row, Col_PortName)->widget())->text();
        m_settings.beginGroup(portName);

        m_settings.setValue("enabled", static_cast<QCheckBox*>(
                                ui->gridLayoutPortsPort->itemAtPosition(row, Col_Enabled)->widget())->isChecked());
        m_settings.setValue("speed", static_cast<BaudrateComboBox*>(
                                ui->gridLayoutPortsPort->itemAtPosition(row, Col_Baud)->widget())->currentBaudrate());
        m_settings.setValue("protocol", static_cast<ProtocolComboBox*>(
                                ui->gridLayoutPortsPort->itemAtPosition(row, Col_Protocol)->widget())->currentProtocol());
        m_settings.setValue("firstAddress", static_cast<QSpinBox*>(
                                ui->gridLayoutPortsPort->itemAtPosition(row, Col_FirstAddress)->widget())->value());
        m_settings.setValue("lastAddress", static_cast<QSpinBox*>(
                                ui->gridLayoutPortsPort->itemAtPosition(row, Col_LastAddress)->widget())->value());
        m_settings.endGroup();
    }

    m_settings.endGroup();
    close();
    emit settingsSaved();
}

bool Settings::getAutoScan()
{
    return ctrl_autoscan;
}

bool Settings::getLock()
{
    return ctrl_locks;
}

bool Settings::getPause()
{
    return ui_pause;
}

int Settings::getFreq()
{
    return ctrl_freq;
}

const QVector<portConfig> & Settings::getSerialPortsConfig()
{
    return serial_ports;
}

const struct portConfig * Settings::getSerialPortConfig(const QString &portPath)
{
    int i = 0;
    for (auto pc: serial_ports)
    {
        if (pc.enabled == true && pc.path == portPath)
        {
            return &serial_ports[i];
        }
        i++;
    }
    return nullptr;
}

void Settings::reloadPorts()
{
    // first remove widgets
    for (int row = 1; row<(serial_ports.count() + 1); row++) {
        for (int col = Col_Enabled; col<=Col_LastAddress; col++) {
            QLayoutItem *item = ui->gridLayoutPortsPort->itemAtPosition(row, col);
            QWidget *widget = item->widget();
            if (widget) {
                ui->gridLayoutPortsPort->removeWidget(widget);
                delete widget;
            }
        }
    }
    serial_ports.clear();

    loadSerialPorts();
    adjustSize();
}
