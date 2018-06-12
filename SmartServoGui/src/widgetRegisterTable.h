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
 * \file widgetRegisterTable.h
 * \date 01/05/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef WIDGET_REGISTER_TABLE_H
#define WIDGET_REGISTER_TABLE_H

#include <QWidget>

namespace Ui {
class widgetRegisterTable;
}

class Servo;

class widgetRegisterTable : public QWidget
{
    Q_OBJECT

    Ui::widgetRegisterTable *ui;

    int tableServoSerieSaved = 0;
    int tableServoModelSaved = 0;

    int getTableIndex(const int servo_serie, const int servo_model, const int reg_name);
    void generateRegisterTableDynamixel(const int servo_serie, const int servo_model);
    void generateRegisterTableHerkuleX(const int servo_serie, const int servo_model);

public:
    explicit widgetRegisterTable(QWidget *parent = 0);
    ~widgetRegisterTable();

    void generateRegisterTable(const int servo_serie, const int servo_model);
    void updateRegisterTableDynamixel(Servo *servo_dxl, const int servoSerie, const int servoModel);
    void updateRegisterTableHerkuleX(Servo *servo_hkx, const int servoSerie, const int servoModel);
    void cleanRegisterTable();

    int getRegisterValueFromTableIndex(const int table_row, const int table_column);
    int getRegisterNameFromTableIndex(const int servo_serie, const int servo_model, int table_index);

public slots:
    void resizeTableWidget();
    void updateTable();

signals:
    void cellChangedSignal(int,int);
};

#endif // WIDGET_REGISTER_TABLE_H
