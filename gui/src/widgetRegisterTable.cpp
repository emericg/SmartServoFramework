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
 * \file widgetRegisterTable.cpp
 * \date 01/05/2017
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "src/widgetRegisterTable.h"
#include "ui_widgetRegisterTable.h"

// SmartServoFramework
#include "../../src/DynamixelController.h"
#include "../../src/HerkuleXController.h"
#include "../../src/DynamixelTools.h"
#include "../../src/HerkuleXTools.h"
#include "../../src/Servo.h"

widgetRegisterTable::widgetRegisterTable(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::widgetRegisterTable)
{
    ui->setupUi(this);
    ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    // Will be connected "on demand"
    //QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));
}

widgetRegisterTable::~widgetRegisterTable()
{
    delete ui;
}

/* ************************************************************************** */

void widgetRegisterTable::generateRegisterTable(const int servo_serie, const int servo_model)
{
    // Remove all rows
    for (int i = ui->tableWidget->rowCount(); i >= 0; i--)
    {
        ui->tableWidget->removeRow(i);
    }

    // Change table format when switching between Dynamixel and HerkuleX devices
    // > Add/remove a new row and change the table legend

    if (servo_serie >= SERVO_HERKULEX)
    {
        if (servo_serie != tableServoSerieSaved)
        {
            ui->tableWidget->setColumnHidden(2, false);
            ui->tableWidget->horizontalHeaderItem(1)->setText(tr("ROM Value"));
            ui->tableWidget->horizontalHeaderItem(2)->setText(tr("RAM Value"));
        }

        // Generate new table
        generateRegisterTableHerkuleX(servo_serie, servo_model);
    }
    else // if (servo_serie >= SERVO_DYNAMIXEL)
    {
        if (servo_serie != tableServoSerieSaved)
        {
            ui->tableWidget->setColumnHidden(2, true);
            ui->tableWidget->horizontalHeaderItem(1)->setText(tr("Value"));
        }

        // Generate new table
        generateRegisterTableDynamixel(servo_serie, servo_model);
    }

    // Save table params
    tableServoSerieSaved = servo_serie;
    tableServoModelSaved = servo_model;

    // Update
    ui->tableWidget->update();
}

void widgetRegisterTable::generateRegisterTableHerkuleX(const int servo_serie, const int servo_model)
{
    int row = 0;
    const int (*ct)[8] = getRegisterTable(servo_serie, servo_model);
    QFont font("Helvetica", 12, QFont::Bold);
    QColor legend(85, 85, 127, 128);
    QColor grey(200, 200, 200, 100);
    QColor white(255, 255, 255, 255);
    QColor green(85, 170, 0, 64);
    QColor orange(255, 170, 0, 64);
    QBrush br_ro(orange, Qt::Dense4Pattern);
    QBrush br_rw(green, Qt::Dense4Pattern);

    // 'EEPROM ONLY'
    ////////////////////////////////////////////////////////////////////////

    // Add 'EEPROM ONLY' legend
    {
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));
        QTableWidgetItem *item_title = new QTableWidgetItem(tr("EEPROM registers"));
        item_title->setFont(font);
        item_title->setTextColor(white);
        item_title->setFlags(item_title->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_title->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 0, item_title);
        QTableWidgetItem *item_spacer1 = new QTableWidgetItem("");
        item_spacer1->setFlags(item_spacer1->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer1->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 1, item_spacer1);
        QTableWidgetItem *item_spacer2 = new QTableWidgetItem("");
        item_spacer2->setFlags(item_spacer2->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer2->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 2, item_spacer2);
        row++;
    }

    // Add 'EEPROM ONLY' registers
    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name = getRegisterName(ct, i);
        if (reg_name < 0)
        {
            //std::cerr << "[ERROR bad index] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_rom = getRegisterAddr(ct, reg_name, REGISTER_ROM);
        if (reg_addr_rom < 0)
        {
            //std::cerr << "[ERROR bad name] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_ram = getRegisterAddr(ct, reg_name, REGISTER_RAM);
        if (reg_addr_ram >= 0)
        {
            //std::cerr << "[ERROR not ROM only register] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }

        {
            ui->tableWidget->insertRow(row);

            std::string regdesc_str = getRegisterDescriptionTxt(reg_name);
            QString regdesc_qstr(regdesc_str.c_str());
            //std::cout << "ROM reg id/" << i << " (of " << getRegisterCount(ct) << ") addr/" << reg_addr << " name/" << regdesc << std::endl;

            QTableWidgetItem *description = new QTableWidgetItem(regdesc_qstr);
            QTableWidgetItem *rom_value = new QTableWidgetItem("");
            QTableWidgetItem *ram_value = new QTableWidgetItem("");
            description->setFlags(description->flags() ^ Qt::ItemIsEditable);
            if (getRegisterAccessMode(ct, reg_name) == READ_ONLY)
            {
                description->setBackgroundColor(orange);
                rom_value->setBackgroundColor(orange);
                rom_value->setTextAlignment(Qt::AlignCenter);
                rom_value->setFlags(rom_value->flags() ^ Qt::ItemIsEditable);
                ram_value->setBackground(br_ro);
                ram_value->setFlags(ram_value->flags() ^ Qt::ItemIsEditable);
            }
            else
            {
                description->setBackgroundColor(green);
                rom_value->setBackgroundColor(green);
                rom_value->setTextAlignment(Qt::AlignCenter);
                ram_value->setBackground(br_rw);
                ram_value->setFlags(ram_value->flags() ^ Qt::ItemIsEditable);
            }
            ui->tableWidget->setItem(row, 0, description);
            ui->tableWidget->setItem(row, 1, rom_value);
            ui->tableWidget->setItem(row, 2, ram_value);

            QString addr;
            addr.setNum(reg_addr_rom, 16);
            addr = addr.toUpper();
            if (addr.size() == 1) { addr.prepend("0x0"); } else { addr.prepend("0x"); }
            ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(addr));

            //
            row++;
        }
    }

    // 'EEPROM' and 'RAM'
    ////////////////////////////////////////////////////////////////////////

    // Add 'EEPROM' and 'RAM' legend
    {
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));
        QTableWidgetItem *item_title = new QTableWidgetItem(tr("Registers"));
        item_title->setFont(font);
        item_title->setTextColor(white);
        item_title->setFlags(item_title->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_title->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 0, item_title);
        QTableWidgetItem *item_spacer1 = new QTableWidgetItem("");
        item_spacer1->setFlags(item_spacer1->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer1->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 1, item_spacer1);
        QTableWidgetItem *item_spacer2 = new QTableWidgetItem("");
        item_spacer2->setFlags(item_spacer2->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer2->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 2, item_spacer2);
        row++;
    }

    // Add 'EEPROM' and 'RAM' registers
    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name = getRegisterName(ct, i);
        if (reg_name < 0)
        {
            //std::cerr << "[ERROR bad index] RAM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_rom = getRegisterAddr(ct, reg_name, REGISTER_ROM);
        if (reg_addr_rom < 0)
        {
            //std::cerr << "[ERROR not ROM/RAM register] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_ram = getRegisterAddr(ct, reg_name, REGISTER_RAM);
        if (reg_addr_ram < 0)
        {
            //std::cerr << "[ERROR not ROM/RAM register] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }

        {
            ui->tableWidget->insertRow(row);

            std::string regdesc_str = getRegisterDescriptionTxt(reg_name);
            QString regdesc_qstr(regdesc_str.c_str());
            //std::cout << "reg id/" << i << " (of " << getRegisterCount(ct) << ") addr/" << reg_addr << " name/" << regdesc << std::endl;

            QTableWidgetItem *description = new QTableWidgetItem(regdesc_qstr);
            QTableWidgetItem *rom_value = new QTableWidgetItem("");
            QTableWidgetItem *ram_value = new QTableWidgetItem("");
            description->setFlags(description->flags() ^ Qt::ItemIsEditable);
            if (getRegisterAccessMode(ct, reg_name) == READ_ONLY)
            {
                description->setBackgroundColor(orange);
                rom_value->setBackgroundColor(grey);
                rom_value->setTextAlignment(Qt::AlignCenter);
                rom_value->setFlags(rom_value->flags() ^ Qt::ItemIsEditable);
                ram_value->setBackgroundColor(orange);
                ram_value->setTextAlignment(Qt::AlignCenter);
                ram_value->setFlags(ram_value->flags() ^ Qt::ItemIsEditable);
            }
            else
            {
                description->setBackgroundColor(green);
                rom_value->setBackgroundColor(green);
                rom_value->setTextAlignment(Qt::AlignCenter);
                ram_value->setBackgroundColor(green);
                ram_value->setTextAlignment(Qt::AlignCenter);
            }

            ui->tableWidget->setItem(row, 0, description);
            ui->tableWidget->setItem(row, 1, rom_value);
            ui->tableWidget->setItem(row, 2, ram_value);
            ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));

            //
            row++;
        }
    }

    // 'RAM ONLY'
    ////////////////////////////////////////////////////////////////////////

    // Add 'RAM ONLY' legend
    {
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));
        QTableWidgetItem *item_title = new QTableWidgetItem(tr("RAM registers"));
        item_title->setFont(font);
        item_title->setTextColor(white);
        item_title->setFlags(item_title->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_title->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 0, item_title);
        QTableWidgetItem *item_spacer1 = new QTableWidgetItem("");
        item_spacer1->setFlags(item_spacer1->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer1->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 1, item_spacer1);
        QTableWidgetItem *item_spacer2 = new QTableWidgetItem("");
        item_spacer2->setFlags(item_spacer2->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer2->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 2, item_spacer2);
        row++;
    }

    // Add 'RAM ONLY' registers
    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name = getRegisterName(ct, i);
        if (reg_name < 0)
        {
            //std::cerr << "[ERROR bad index] RAM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_rom = getRegisterAddr(ct, reg_name, REGISTER_ROM);
        if (reg_addr_rom >= 0)
        {
            //std::cerr << "[ERROR bad name] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr_ram = getRegisterAddr(ct, reg_name, REGISTER_RAM);
        if (reg_addr_ram < 0)
        {
            //std::cerr << "[ERROR not RAM only register] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }

        {
            ui->tableWidget->insertRow(row);

            std::string regdesc_str = getRegisterDescriptionTxt(reg_name);
            QString regdesc_qstr(regdesc_str.c_str());
            //std::cout << "RAM reg id/" << i << " (of " << getRegisterCount(ct) << ") addr/" << reg_addr << " name/" << regdesc << std::endl;

            QTableWidgetItem *description = new QTableWidgetItem(regdesc_qstr);
            QTableWidgetItem *rom_value = new QTableWidgetItem("");
            QTableWidgetItem *ram_value = new QTableWidgetItem("");
            description->setFlags(description->flags() ^ Qt::ItemIsEditable);
            if (getRegisterAccessMode(ct, reg_name) == READ_ONLY)
            {
                description->setBackgroundColor(orange);
                rom_value->setBackground(br_ro);
                rom_value->setFlags(rom_value->flags() ^ Qt::ItemIsEditable);
                ram_value->setBackgroundColor(orange);
                ram_value->setTextAlignment(Qt::AlignCenter);
                ram_value->setFlags(ram_value->flags() ^ Qt::ItemIsEditable);
            }
            else
            {
                description->setBackgroundColor(green);
                rom_value->setBackground(br_rw);
                rom_value->setFlags(rom_value->flags() ^ Qt::ItemIsEditable);
                ram_value->setBackgroundColor(green);
                ram_value->setTextAlignment(Qt::AlignCenter);
            }

            ui->tableWidget->setItem(row, 0, description);
            ui->tableWidget->setItem(row, 1, rom_value);
            ui->tableWidget->setItem(row, 2, ram_value);

            QString addr;
            addr.setNum(reg_addr_ram, 16);
            addr = addr.toUpper();
            if (addr.size() == 1) { addr.prepend("0x0"); } else { addr.prepend("0x"); }
            ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(addr));

            //
            row++;
        }
    }
    //std::cout << "2] Row count (ADDED): " << ui->tableWidget->rowCount() << std::endl;
}

void widgetRegisterTable::generateRegisterTableDynamixel(const int servo_serie, const int servo_model)
{
    int row = 0;
    const int (*ct)[8] = getRegisterTable(servo_serie, servo_model);
    QFont font("Helvetica", 12, QFont::Bold);
    QColor legend(85, 85, 127, 128);
    QColor white(255, 255, 255, 255);
    QColor green(85, 170, 0, 64);
    QColor orange(255, 170, 0, 64);

    // 'EEPROM'
    ////////////////////////////////////////////////////////////////////////

    // Add 'EEPROM' row legend
    {
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));
        QTableWidgetItem *item_title = new QTableWidgetItem(tr("EEPROM registers"));
        item_title->setFont(font);
        item_title->setTextColor(white);
        item_title->setFlags(item_title->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_title->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 0, item_title);
        QTableWidgetItem *item_spacer1 = new QTableWidgetItem("");
        item_spacer1->setFlags(item_spacer1->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer1->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 1, item_spacer1);
        row++;
    }

    // Add ROM row registers
    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name = getRegisterName(ct, i);
        if (reg_name < 0)
        {
            //std::cerr << "[ERROR bad index] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr = getRegisterAddr(ct, reg_name, REGISTER_ROM);
        if (reg_addr < 0)
        {
            //std::cerr << "[ERROR bad name] ROM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }

        {
            ui->tableWidget->insertRow(row);

            std::string regdesc_str = getRegisterDescriptionTxt(reg_name);
            QString regdesc_qstr(regdesc_str.c_str());
            //std::cout << "ROM reg id/" << i << " (of " << getRegisterCount(ct) << ") addr/" << reg_addr << " name/" << regdesc << std::endl;

            QTableWidgetItem *description = new QTableWidgetItem(regdesc_qstr);
            QTableWidgetItem *rom_value = new QTableWidgetItem("");
            description->setFlags(description->flags() ^ Qt::ItemIsEditable);
            if (getRegisterAccessMode(ct, reg_name) == READ_ONLY)
            {
                description->setBackgroundColor(orange);
                rom_value->setBackgroundColor(orange);
                rom_value->setTextAlignment(Qt::AlignCenter);
                rom_value->setFlags(rom_value->flags() ^ Qt::ItemIsEditable);
            }
            else
            {
                description->setBackgroundColor(green);
                rom_value->setBackgroundColor(green);
                rom_value->setTextAlignment(Qt::AlignCenter);
            }
            ui->tableWidget->setItem(row, 0, description);
            ui->tableWidget->setItem(row, 1, rom_value);

            QString addr;
            addr.setNum(reg_addr, 16);
            addr = addr.toUpper();
            if (addr.size() == 1) { addr.prepend("0x0"); } else { addr.prepend("0x"); }
            ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(addr));

            //
            row++;
        }
    }

    // 'RAM'
    ////////////////////////////////////////////////////////////////////////

    // Add 'RAM' row legend
    {
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(""));
        QTableWidgetItem *item_title = new QTableWidgetItem(tr("RAM registers"));
        item_title->setFont(font);
        item_title->setTextColor(white);
        item_title->setFlags(item_title->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_title->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 0, item_title);
        QTableWidgetItem *item_spacer1 = new QTableWidgetItem("");
        item_spacer1->setFlags(item_spacer1->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        item_spacer1->setBackgroundColor(legend);
        ui->tableWidget->setItem(row, 1, item_spacer1);
        row++;
    }

    // Add RAM registers
    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name = getRegisterName(ct, i);
        if (reg_name < 0)
        {
            //std::cerr << "[ERROR bad index] RAM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }
        int reg_addr = getRegisterAddr(ct, reg_name, REGISTER_RAM);
        if (reg_addr < 0)
        {
            //std::cerr << "[ERROR bad name] RAM reg id/" << i << " addr/" << -1 << std::endl;
            continue;
        }

        {
            ui->tableWidget->insertRow(row);

            std::string regdesc_str = getRegisterDescriptionTxt(reg_name);
            QString regdesc_qstr(regdesc_str.c_str());
            //std::cout << "RAM reg id/" << i << " (of " << getRegisterCount(ct) << ") addr/" << reg_addr << " name/" << regdesc << std::endl;

            QTableWidgetItem *description = new QTableWidgetItem(regdesc_qstr);
            QTableWidgetItem *ram_value = new QTableWidgetItem("");
            description->setFlags(description->flags() ^ Qt::ItemIsEditable);
            if (getRegisterAccessMode(ct, reg_name) == READ_ONLY)
            {
                description->setBackgroundColor(orange);
                ram_value->setBackgroundColor(orange);
                ram_value->setTextAlignment(Qt::AlignCenter);
                ram_value->setFlags(ram_value->flags() ^ Qt::ItemIsEditable);
            }
            else
            {
                description->setBackgroundColor(green);
                ram_value->setBackgroundColor(green);
                ram_value->setTextAlignment(Qt::AlignCenter);
            }

            ui->tableWidget->setItem(row, 0, description);
            ui->tableWidget->setItem(row, 1, ram_value);

            QString addr;
            addr.setNum(reg_addr, 16);
            addr = addr.toUpper();
            if (addr.size() == 1) { addr.prepend("0x0"); } else { addr.prepend("0x"); }
            ui->tableWidget->setVerticalHeaderItem(row, new QTableWidgetItem(addr));

            //
            row++;
        }
    }
    //std::cout << "2] Row count (ADDED): " << ui->tableWidget->rowCount() << std::endl;
}

/* ************************************************************************** */

void widgetRegisterTable::cleanRegisterTable()
{
    QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));

    for (int i = 0; i < ui->tableWidget->rowCount(); i++)
    {
        ui->tableWidget->item(i, 1)->setText("");
        if (tableServoSerieSaved >= SERVO_HERKULEX)
        {
            ui->tableWidget->item(i, 2)->setText("");
        }
    }

    QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));
}

void widgetRegisterTable::resizeTableWidget()
{
/*
    // FIXME // does not produce expected results
    int tabWidgetSize = ui->tabWidget->size().width();
    ui->frame_quickcontrols->setMaximumWidth(tabWidgetSize * 0.40);
    ui->tableWidget->setMaximumWidth(tabWidgetSize * 0.60);
*/
    int error_margin = 16; // (column count) * (grid line size) + (scroll bar size) ?
    int width_available = ui->tableWidget->size().width();
    int width_header = ui->tableWidget->verticalHeader()->size().width();

    // Scale register table
    if (tableServoSerieSaved >= SERVO_HERKULEX)
    {
        int width_value_col = 90;
        int width_description_col = width_available - width_header - width_value_col*2 - error_margin;
        ui->tableWidget->setColumnWidth(0, width_description_col);
        ui->tableWidget->setColumnWidth(1, width_value_col);
        ui->tableWidget->setColumnWidth(2, width_value_col);
    }
    else
    {
        int width_value_col = 100;
        int width_description_col = width_available - width_header - width_value_col - error_margin;
        ui->tableWidget->setColumnWidth(0, width_description_col);
        ui->tableWidget->setColumnWidth(1, width_value_col);
    }
}

void widgetRegisterTable::updateTable()
{
    ui->tableWidget->update();
}

/* ************************************************************************** */

void widgetRegisterTable::updateRegisterTableHerkuleX(Servo *servo_hkx, const int servoSerie, const int servoModel)
{
    ServoHerkuleX *servo = static_cast <ServoHerkuleX *>(servo_hkx);

    QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));

    // EEPROM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MODEL_NUMBER),1)->setText(QString::fromStdString(servo->getModelString()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_FIRMWARE_VERSION),1)->setText(QString::number(servo->getFirmwareVersion()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_BAUD_RATE),1)->setText(QString::number(servo->getBaudRate()));
//RESERVED

    // EEPROM and RAM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ID),1)->setText(QString::number(servo->getValue(REG_ID, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ID),2)->setText(QString::number(servo->getValue(REG_ID, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STATUS_RETURN_LEVEL),1)->setText(QString::number(servo->getStatusReturnLevel(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STATUS_RETURN_LEVEL),2)->setText(QString::number(servo->getStatusReturnLevel(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_LED),1)->setText(QString::number(servo->getAlarmLed(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_LED),2)->setText(QString::number(servo->getAlarmLed(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_SHUTDOWN),1)->setText(QString::number(servo->getAlarmShutdown(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_SHUTDOWN),2)->setText(QString::number(servo->getAlarmShutdown(REGISTER_RAM)));
//RESERVED
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TEMPERATURE_LIMIT),1)->setText(QString::number(servo->getHighestLimitTemp(REGISTER_ROM), 'g', 3) + " *");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TEMPERATURE_LIMIT),2)->setText(QString::number(servo->getHighestLimitTemp(REGISTER_RAM), 'g', 3) + " *");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_LOWEST_LIMIT),1)->setText(QString::number(servo->getLowestLimitVolt(REGISTER_ROM), 'g', 3) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_LOWEST_LIMIT),2)->setText(QString::number(servo->getLowestLimitVolt(REGISTER_RAM), 'g', 3) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_HIGHEST_LIMIT),1)->setText(QString::number(servo->getHighestLimitVolt(REGISTER_ROM), 'g', 3) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_HIGHEST_LIMIT),2)->setText(QString::number(servo->getHighestLimitVolt(REGISTER_RAM), 'g', 3) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ACCEL_RATIO),1)->setText(QString::number(servo->getValue(REG_ACCEL_RATIO, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ACCEL_RATIO),2)->setText(QString::number(servo->getValue(REG_ACCEL_RATIO, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_ACCEL_TIME),1)->setText(QString::number(servo->getValue(REG_MAX_ACCEL_TIME, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_ACCEL_TIME),2)->setText(QString::number(servo->getValue(REG_MAX_ACCEL_TIME, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DEAD_ZONE),1)->setText(QString::number(servo->getValue(REG_DEAD_ZONE, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DEAD_ZONE),2)->setText(QString::number(servo->getValue(REG_DEAD_ZONE, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_SATURATOR_OFFSET),1)->setText(QString::number(servo->getValue(REG_SATURATOR_OFFSET, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_SATURATOR_OFFSET),2)->setText(QString::number(servo->getValue(REG_SATURATOR_OFFSET, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_SATURATOR_SLOPE),1)->setText(QString::number(servo->getValue(REG_SATURATOR_SLOPE, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_SATURATOR_SLOPE),2)->setText(QString::number(servo->getValue(REG_SATURATOR_SLOPE, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_OFFSET),1)->setText(QString::number(servo->getValue(REG_PWM_OFFSET, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_OFFSET),2)->setText(QString::number(servo->getValue(REG_PWM_OFFSET, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_MIN),1)->setText(QString::number(servo->getValue(REG_PWM_MIN, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_MIN),2)->setText(QString::number(servo->getValue(REG_PWM_MIN, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_MAX),1)->setText(QString::number(servo->getValue(REG_PWM_MAX, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_MAX),2)->setText(QString::number(servo->getValue(REG_PWM_MAX, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_OVERLOAD_THRESHOLD),1)->setText(QString::number(servo->getValue(REG_PWM_OVERLOAD_THRESHOLD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM_OVERLOAD_THRESHOLD),2)->setText(QString::number(servo->getValue(REG_PWM_OVERLOAD_THRESHOLD, REGISTER_RAM)));

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),1)->setText(QString::number(servo->getCwAngleLimit(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),2)->setText(QString::number(servo->getCwAngleLimit(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),1)->setText(QString::number(servo->getCcwAngleLimit(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),2)->setText(QString::number(servo->getCcwAngleLimit(REGISTER_RAM)));

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_D_GAIN),1)->setText(QString::number(servo->getDGain(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_D_GAIN),2)->setText(QString::number(servo->getDGain(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_I_GAIN),1)->setText(QString::number(servo->getIGain(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_I_GAIN),2)->setText(QString::number(servo->getIGain(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_P_GAIN),1)->setText(QString::number(servo->getPGain(REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_P_GAIN),2)->setText(QString::number(servo->getPGain(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_POS_FEED_FRW_1st_GAIN),1)->setText(QString::number(0));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_POS_FEED_FRW_1st_GAIN),2)->setText(QString::number(0));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_POS_FEED_FRW_2nd_GAIN),1)->setText(QString::number(0));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_POS_FEED_FRW_2nd_GAIN),2)->setText(QString::number(0));
    if (servoModel == SERVO_DRS_0402 || servoModel == SERVO_DRS_0602)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VELOCITY_KP),2)->setText(QString::number(servo->getValue(REG_VELOCITY_KP)));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VELOCITY_KI),2)->setText(QString::number(servo->getValue(REG_VELOCITY_KI)));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED_BLINKING),1)->setText(QString::number(servo->getValue(REG_LED_BLINKING, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED_BLINKING),2)->setText(QString::number(servo->getValue(REG_LED_BLINKING, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ADC_FAULT_CHECK_PRD),1)->setText(QString::number(servo->getValue(REG_ADC_FAULT_CHECK_PRD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ADC_FAULT_CHECK_PRD),2)->setText(QString::number(servo->getValue(REG_ADC_FAULT_CHECK_PRD, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PKT_GARBAGE_CHECK_PRD),1)->setText(QString::number(servo->getValue(REG_PKT_GARBAGE_CHECK_PRD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PKT_GARBAGE_CHECK_PRD),2)->setText(QString::number(servo->getValue(REG_PKT_GARBAGE_CHECK_PRD, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STOP_DETECTION_PRD),1)->setText(QString::number(servo->getValue(REG_STOP_DETECTION_PRD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STOP_DETECTION_PRD),2)->setText(QString::number(servo->getValue(REG_STOP_DETECTION_PRD, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_OVERLOAD_DETECTION_PRD),1)->setText(QString::number(servo->getValue(REG_OVERLOAD_DETECTION_PRD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_OVERLOAD_DETECTION_PRD),2)->setText(QString::number(servo->getValue(REG_OVERLOAD_DETECTION_PRD, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STOP_THRESHOLD),1)->setText(QString::number(servo->getValue(REG_STOP_THRESHOLD, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STOP_THRESHOLD),2)->setText(QString::number(servo->getValue(REG_STOP_THRESHOLD, REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_INPOSITION_MARGIN),1)->setText(QString::number(servo->getValue(REG_INPOSITION_MARGIN, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_INPOSITION_MARGIN),2)->setText(QString::number(servo->getValue(REG_INPOSITION_MARGIN, REGISTER_RAM)));
//RESERVED
//RESERVED
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CALIBRATION_DIFFERENCE),1)->setText(QString::number(servo->getValue(REG_CALIBRATION_DIFFERENCE, REGISTER_ROM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CALIBRATION_DIFFERENCE),2)->setText(QString::number(servo->getValue(REG_CALIBRATION_DIFFERENCE, REGISTER_RAM)));

    //RAM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STATUS_ERROR),2)->setText(QString::number(servo->getValue(REG_STATUS_ERROR)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STATUS_DETAIL),2)->setText(QString::number(servo->getValue(REG_STATUS_DETAIL)));
    if (servoModel == SERVO_DRS_0402 || servoModel == SERVO_DRS_0602)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_AUX_1),2)->setText(QString::number(servo->getValue(REG_AUX_1)));
    }
//RESERVED
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_ENABLE),2)->setText(QString::number(servo->getValue(REG_TORQUE_ENABLE)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED),2)->setText(QString::number(servo->getValue(REG_LED)));

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_TEMPERATURE),2)->setText(QString::number(servo->getCurrentTemperature(), 'g', 3) + " *");

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_VOLTAGE),2)->setText(QString::number(servo->getCurrentVoltage(), 'g', 3) + " V");

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_CONTROL_MODE),2)->setText(QString::number(servo->getValue(REG_CURRENT_CONTROL_MODE)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TICK),2)->setText(QString::number(servo->getValue(REG_TICK)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CALIBRATED_POSITION),2)->setText(QString::number(servo->getValue(REG_CALIBRATED_POSITION)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_POSITION),2)->setText(QString::number(servo->getValue(REG_ABSOLUTE_POSITION)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DIFFERENTIAL_POSITION),2)->setText(QString::number(servo->getValue(REG_DIFFERENTIAL_POSITION)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM),2)->setText(QString::number(servo->getValue(REG_PWM)));
    if (servoModel == SERVO_DRS_0402 || servoModel == SERVO_DRS_0602)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_2nd_POSITION),2)->setText(QString::number(servo->getValue(REG_ABSOLUTE_2nd_POSITION)));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_GOAL_POSITION),2)->setText(QString::number(servo->getValue(REG_ABSOLUTE_GOAL_POSITION)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_TRAJECTORY),2)->setText(QString::number(servo->getValue(REG_GOAL_TRAJECTORY)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_VELOCITY),2)->setText(QString::number(servo->getValue(REG_GOAL_VELOCITY)));

    QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));
}

void widgetRegisterTable::updateRegisterTableDynamixel(Servo *servo_dxl, const int servoSerie, const int servoModel)
{
    ServoDynamixel *servo = static_cast <ServoDynamixel *>(servo_dxl);

    QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));

    // EEPROM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MODEL_NUMBER),1)->setText(QString::fromStdString(servo->getModelString()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_FIRMWARE_VERSION),1)->setText(QString::number(servo->getFirmwareVersion()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ID),1)->setText(QString::number(servo->getValue(REG_ID)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_BAUD_RATE),1)->setText(QString::number(servo->getBaudRate()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_RETURN_DELAY_TIME),1)->setText(QString::number(servo->getReturnDelay()));

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),1)->setText(QString::number(servo->getCwAngleLimit()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),1)->setText(QString::number(servo->getCcwAngleLimit()));

    if (servoSerie == SERVO_EX || servoModel == SERVO_MX106)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DRIVE_MODE),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getDriveMode()));
    }
    else if (servoSerie == SERVO_XL || servoModel == SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CONTROL_MODE),1)->setText(QString::number(static_cast<ServoXL*>(servo)->getControlMode()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TEMPERATURE_LIMIT),1)->setText(QString::number(servo->getHighestLimitTemp()) + " *");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_LOWEST_LIMIT),1)->setText(QString::number(servo->getLowestLimitVolt()) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_HIGHEST_LIMIT),1)->setText(QString::number(servo->getHighestLimitVolt()) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_TORQUE),1)->setText(QString::number(servo->getMaxTorque()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_STATUS_RETURN_LEVEL),1)->setText(QString::number(servo->getStatusReturnLevel()));
    if (servoModel != SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_LED),1)->setText(QString::number(servo->getAlarmLed()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ALARM_SHUTDOWN),1)->setText(QString::number(servo->getAlarmShutdown()));

    if (servoSerie == SERVO_MX)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MULTI_TURN_OFFSET),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getMultiTurnOffset()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_RESOLUTION_DIVIDER),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getResolutionDivider()));
    }

    // RAM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_ENABLE),1)->setText(QString::number(servo->getTorqueEnabled()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED),1)->setText(QString::number(servo->getLed()));
    if (servoSerie == SERVO_AX || servoSerie == SERVO_DX ||
        servoSerie == SERVO_RX || servoSerie == SERVO_EX)
    {
        // Only on AX, DX, RX and EX
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CW_COMPLIANCE_MARGIN),1)->setText(QString::number(static_cast<ServoAX*>(servo)->getCwComplianceMargin()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CCW_COMPLIANCE_MARGIN),1)->setText(QString::number(static_cast<ServoAX*>(servo)->getCcwComplianceMargin()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CW_COMPLIANCE_SLOPE),1)->setText(QString::number(static_cast<ServoAX*>(servo)->getCwComplianceSlope()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CCW_COMPLIANCE_SLOPE),1)->setText(QString::number(static_cast<ServoAX*>(servo)->getCcwComplianceSlope()));
    }
    else
    {
        // Only on MX and XL-320
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_D_GAIN),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getDGain()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_I_GAIN),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getIGain()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_P_GAIN),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getPGain()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_POSITION),1)->setText(QString::number(servo->getGoalPosition()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_SPEED),1)->setText(QString::number(servo->getMovingSpeed()));
    if (servoModel != SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_LIMIT),1)->setText(QString::number(servo->getTorqueLimit()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_POSITION),1)->setText(QString::number(servo->getCurrentPosition()));

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_SPEED),1)->setText(QString::number(servo->getCurrentSpeed()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_LOAD),1)->setText(QString::number(servo->getCurrentLoad()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_VOLTAGE),1)->setText(QString::number(servo->getCurrentVoltage()) + " V");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_TEMPERATURE),1)->setText(QString::number(servo->getCurrentTemperature()) + " *");

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_REGISTERED),1)->setText(QString::number(servo->getRegistered()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MOVING),1)->setText(QString::number(servo->getMoving()));
    if (servoModel != SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LOCK),1)->setText(QString::number(servo->getLock()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PUNCH),1)->setText(QString::number(servo->getPunch()));

    if (servoSerie == SERVO_EX)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_CURRENT),1)->setText(QString::number(static_cast<ServoEX*>(servo)->getSensedCurrent()));
    }
    else if (servoSerie == SERVO_MX)
    {
        if (servoModel == SERVO_MX64 || servoModel == SERVO_MX106)
        {
            ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_CURRENT),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getConsumingCurrent()));
            ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CONTROL_MODE),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getTorqueControlMode()));
            ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_TORQUE),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getGoalTorque()));
        }
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_ACCELERATION),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getGoalAccel()));
    }
    else if (servoSerie == SERVO_XL || servoModel == SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_TORQUE),1)->setText(QString::number(static_cast<ServoXL*>(servo)->getGoalTorque()));
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_HW_ERROR_STATUS),1)->setText(QString::number(static_cast<ServoXL*>(servo)->getHardwareErrorStatus()));
    }

    QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SIGNAL(cellChangedSignal(int, int)));
}

/* ************************************************************************** */

int widgetRegisterTable::getTableIndex(const int servo_serie, const int servo_model, const int reg_name)
{
    const int (*ct)[8] = getRegisterTable(servo_serie, servo_model);
    int index = getRegisterTableIndex(ct, reg_name);
    int addr_rom = getRegisterAddr(ct, reg_name, REGISTER_ROM);
    int addr_ram = getRegisterAddr(ct, reg_name, REGISTER_RAM);

    if (index > -1)
    {
        if (servo_serie >= SERVO_HERKULEX)
        {
            if (addr_rom > -1 && addr_ram < 0)
            {
                index += 1;
            }
            else if (addr_rom > -1 && addr_ram > -1)
            {
                index += 2;
            }
            else if (addr_rom < 0 && addr_ram > -1)
            {
                index += 3;
            }
        }
        else //if (servo_serie >= SERVO_DYNAMIXEL)
        {
            if (addr_rom > -1)
            {
                index += 1;
            }
            else if (addr_ram > -1)
            {
                index += 2;
            }
        }
    }
    else
    {
        index = 0; // Fallback and write in the first row (should be EEPROM legend)
    }

    return index;
}

int widgetRegisterTable::getRegisterValueFromTableIndex(const int table_row, const int table_column)
{
    return ui->tableWidget->item(table_row, table_column)->text().toInt();
}

int widgetRegisterTable::getRegisterNameFromTableIndex(const int servo_serie, const int servo_model, int table_index)
{
    int reg_name = -1;

    // Perform reverse lookup
    const int (*ct)[8] = getRegisterTable(servo_serie, servo_model);

    for (int i = 0; i < static_cast<int>(getRegisterCount(ct)); i++)
    {
        int reg_name_tmp = getRegisterName(ct, i);

        if (table_index == getTableIndex(servo_serie, servo_model, reg_name_tmp))
        {
            reg_name = reg_name_tmp;
            break;
        }
    }

    return reg_name;
}

/* ************************************************************************** */
