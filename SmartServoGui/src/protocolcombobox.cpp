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
#include "protocolcombobox.h"

ProtocolComboBox::ProtocolComboBox(QWidget *parent) :
    QComboBox(parent)
{
    addItem(tr("Dynamixel v1"), ServoProtocol::PROTOCOL_DXLv1);
    addItem(tr("Dynamixel v2"), ServoProtocol::PROTOCOL_DXLv2);
    addItem(tr("HerkuleX"), ServoProtocol::PROTOCOL_HKX);
}

ServoProtocol ProtocolComboBox::currentProtocol() const
{
    return static_cast<ServoProtocol>(currentData().toInt());
}

void ProtocolComboBox::setCurrentProcotol(const ServoProtocol protocol)
{
    setCurrentIndex(findData(protocol));
}
