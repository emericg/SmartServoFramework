/*!
 * This file is part of SmartServoFramework.
 * Copyright (c) 2019, INRIA, All rights reserved.
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
 * \file protocolcombobox.cpp
 * \date 25/03/2019
 * \author Miklos Marton <martonmiklosqdev@gmail.com>
 */
#ifndef BAUDRATECOMBOBOX_H
#define BAUDRATECOMBOBOX_H

#include <QComboBox>
#include <QIntValidator>

class BaudrateComboBox : public QComboBox
{
public:
    BaudrateComboBox(QWidget *parent = nullptr);
    qint32 currentBaudrate();
    void setBaudrate(qint32 baud);
private:
    QIntValidator *m_validator;
};

#endif // BAUDRATECOMBOBOX_H
