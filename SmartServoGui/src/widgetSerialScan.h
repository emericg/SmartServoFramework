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
 * \file widgetSerialScan.h
 * \date 03/05/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef WIDGET_SERIAL_SCAN_H
#define WIDGET_SERIAL_SCAN_H

#include "SmartServoFramework/ServoTools.h"

#include <QSettings>
#include <QWidget>

namespace Ui {
class widgetSerialScan;
}

class widgetSerialScan : public QWidget
{
    Q_OBJECT
    enum PortMode {
        Auto,
        Manual,

        DXL_V1_1Mb,
        DXL_V1_115k,
        DXL_V1_57p6k,

        DXL_V2_1Mb,
        DXL_V2_115k,
        DXL_V2_57p6k,

        HKX_1Mb,
        HKX_115k,
        HKX_57p6k,

        Saved
    };

    Ui::widgetSerialScan *ui;

    ServoProtocol saved_protocol = ServoProtocol::PROTOCOL_UNKNOWN;
    int saved_speed = -1;

    QSettings m_settings;

public:
    explicit widgetSerialScan(QString &port, QWidget *parent = nullptr);
    ~widgetSerialScan();

    void setSavedParameters(ServoProtocol protocol, int speed, int firstAddress = 0, int lastAddress = 253);
    void loadSavedParameters();

    std::string getDeviceName();
    bool isSelected();

    int getCurrentIndex();

    int getCurrentSpeed(int index = -1);
    ServoProtocol getCurrentProtocol(int index = -1);
    int getCurrentDeviceClass(int index = -1);
    int getMinRange();
    int getMaxRange();

private slots:
    void on_portMode_currentIndexChanged(int index);
    void on_portScanButton_clicked();
    void on_rangeStop_spinBox_valueChanged(int arg1);
    void on_rangeStart_spinBox_valueChanged(int arg1);

signals:
    void scanButton(QString port);
};

#endif // WIDGET_SERIAL_SCAN_H
