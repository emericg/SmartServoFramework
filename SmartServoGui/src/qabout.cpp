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
 * \file qabout.cpp
 * \date 23/09/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "qabout.h"
#include "ui_qabout.h"

#include <qdesktopservices.h>

QAbout::QAbout(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QAbout)
{
    ui->setupUi(this);
    connect(ui->pushButton_ok, SIGNAL(clicked()), this, SLOT(close()));
    connect(ui->pushButton_website, SIGNAL(clicked()), this, SLOT(openWebsite()));
}

QAbout::~QAbout()
{
    delete ui;
}

void QAbout::openWebsite()
{
    QString link = "https://github.com/emericg/SmartServoFramework";
    QDesktopServices::openUrl(QUrl(link));
}
