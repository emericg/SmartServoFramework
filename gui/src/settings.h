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
 * \file settings.h
 * \date 05/11/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <QWidget>

#include "rapidjson/document.h"

namespace Ui {
class Settings;
}

struct portconfig_s
{
    bool on;
    std::string path;
    std::string protocol;
    int speed;
};

class Settings : public QWidget
{
    Q_OBJECT

    Ui::Settings *ui;

    //! Path of the config file (OS dependant)
    std::string filepath;

    //! Config file parser
    rapidjson::Document *parser;

    // Settings
    bool ui_pause;
    bool ctrl_autoscan;
    bool ctrl_locks;
    int ctrl_freq;
    std::vector <struct portconfig_s> serial_ports;

private slots:
    void exitSettings();
    void saveSettings();

public slots:
    void loadSettings();

public:
    explicit Settings(QWidget *parent = 0);
    ~Settings();

    bool getAutoScan();
    bool getLock();
    bool getPause();
    int getFreq();
    std::vector <struct portconfig_s> getSerialPortsConfig();

    int readSettings();
    int writeSettings();
};

#endif // SETTINGS_H
