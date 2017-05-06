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
 * \file widgetSerialError.h
 * \date 22/04/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef WIDGET_SERIAL_ERROR_H
#define WIDGET_SERIAL_ERROR_H

#include <QWidget>

namespace Ui {
class widgetSerialError;
}

class ControllerAPI;
class Servo;

class widgetSerialError : public QWidget
{
    Q_OBJECT

    Ui::widgetSerialError *ui;

public:
    explicit widgetSerialError(QWidget *parent = 0);
    ~widgetSerialError();

    void updateVisibility(const int servoSerie);

    void handleErrors(ControllerAPI *ctrl, Servo *servo,
                      const int servoSerie, const int servoModel);

signals:
    void clearButton();
    void updateButton();
};

#endif // WIDGET_SERIAL_ERROR_H
