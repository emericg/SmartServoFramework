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
 * \file qabout.h
 * \date 23/09/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef QABOUT_H
#define QABOUT_H

#include <QDialog>

namespace Ui {
class QAbout;
}

class QAbout : public QDialog
{
    Q_OBJECT

    Ui::QAbout *ui;

public:
    explicit QAbout(QWidget *parent = 0);
    ~QAbout();

private slots:
    void openWebsite();
};

#endif // QABOUT_H
