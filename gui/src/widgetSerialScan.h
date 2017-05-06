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

#include <QWidget>

namespace Ui {
class widgetSerialScan;
}

class ControllerAPI;
class Servo;

class widgetSerialScan : public QWidget
{
    Q_OBJECT

    Ui::widgetSerialScan *ui;

public:
    explicit widgetSerialScan(QString &port, QWidget *parent = 0);
    ~widgetSerialScan();

    std::string getDeviceName();
    bool isSelected();

    int getCurrentIndex();

    int getCurrentSpeed(int index = -1);
    int getCurrentProtocol(int index = -1);
    int getCurrentDeviceClass(int index = -1);
    int getMinRange();
    int getMaxRange();

public slots:
    void on_portMode_currentIndexChanged(int index);
    void on_portScanButton_clicked();

signals:
    void scanButton(QString port);
};

#endif // WIDGET_SERIAL_SCAN_H
