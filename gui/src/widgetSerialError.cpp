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

widgetSerialError::widgetSerialError(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::widgetSerialError)
{
    ui->setupUi(this);
}

widgetSerialError::~widgetSerialError()
{
    delete ui;
}
