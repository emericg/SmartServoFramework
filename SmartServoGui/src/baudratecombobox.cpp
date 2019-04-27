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
#include "baudratecombobox.h"

#include <QSerialPortInfo>

BaudrateComboBox::BaudrateComboBox(QWidget *parent) :
    QComboBox(parent)
{
    setEditable(true);
    QList<qint32> bauds = QSerialPortInfo::standardBaudRates();
    std::sort(bauds.begin(), bauds.end());
    for (qint32 baud :bauds ) {
        if (baud >= 1200)
            addItem(QString::number(baud));
    }

    m_validator = new QIntValidator(1200, 4500000, this);
    setValidator(m_validator);
}

qint32 BaudrateComboBox::currentBaudrate()
{
    return currentText().toInt();
}

void BaudrateComboBox::setBaudrate(qint32 baud)
{
    // try to find an item for the baudrate
    // if not found insert it to the list at
    // the proper position
    int index = findText(QString::number(baud));
    for (int i = 0; i<count(); i++) {
        if (baud == itemText(i).toInt()) {
            setCurrentIndex(i);
            return;
        } else if (baud < itemText(i).toInt()) {
            insertItem(i, QString::number(baud));
            setCurrentIndex(i);
            return;
        }
    }
    addItem(QString::number(baud));
}
