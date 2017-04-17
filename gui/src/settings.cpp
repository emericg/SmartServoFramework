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
 * \file settings.cpp
 * \date 05/11/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "settings.h"
#include "ui_settings.h"

#include "thirdparty/rapidjson/filereadstream.h"
#include "thirdparty/rapidjson/filewritestream.h"
#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/prettywriter.h"

#if defined(__linux__) || defined(__gnu_linux)
#include <sys/types.h>
#include <sys/stat.h>
#endif

#include <iostream>
#include <fstream>

using namespace rapidjson;

Settings::Settings(QWidget *parent):
    QWidget(parent),
    ui(new Ui::Settings)
{
    // UI
    ui->setupUi(this);

    // Connect various UI slots
    QObject::connect(ui->pushButton_exit, SIGNAL(clicked()), this, SLOT(exitSettings()));
    QObject::connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(saveSettings()));

    // Init json parser
    parser = new Document();

    // Configuration file path
    filepath.clear();

#if defined(__linux__) || defined(__gnu_linux)
    char *env = getenv("HOME");
    if (env)
    {
        filepath  = env;
        filepath += "/.config/SmartServoGUI";
        mkdir(filepath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        filepath += "/settings.json";
    }
#elif defined(_WIN32) || defined(_WIN64)
    char *env = getenv("%APPDATA%");
    if (env)
    {
        filepath  = env;
        filepath += "/SmartServoGUI";
        _mkdir(filepath.c_str());
        filepath = "/SmartServoGUI.json";
    }
#elif defined(__APPLE__) || defined(__MACH__)
    char *env = getenv("HOME");
    if (env)
    {
        filepath  = env;
        filepath += "/Library/Preferences/SmartServoGUI.json";
    }
#else
    #error "No compatible operating system detected!"
#endif
}

Settings::~Settings()
{
    delete ui;
}

void Settings::loadSettings()
{
    // settings
    ui->checkBox_pause->setChecked(ui_pause);
    ui->checkBox_autoscan->setChecked(ctrl_autoscan);
    ui->checkBox_locks->setChecked(ctrl_locks);
    ui->spinBox_freq->setValue(ctrl_freq);
/*
    // serial ports
    ui->checkBox_0->setChecked();
    ui->lineEdit_0->setText();
    ui->comboBox_0->setCurrentIndex()
    ui->spinBox_0->setValue();
*/
}

void Settings::exitSettings()
{
    close();
}

void Settings::saveSettings()
{
    std::cout << "saveSettings() " << std::endl;
    int write = 0;

    if (ui->checkBox_pause->isChecked() != ui_pause)
    {
        if ((*parser).HasMember("ui") == false)
        {
            Value contacts(kObjectType);
            (*parser).AddMember("ui", contacts, parser->GetAllocator());
        }
        if ((*parser)["ui"].HasMember("pause") == false)
        {
            (*parser)["ui"].AddMember("pause", ui_pause, parser->GetAllocator());
        }

        ui_pause = ui->checkBox_pause->isChecked();
        (*parser)["ui"]["pause"].SetBool(ui_pause);
        write++;
    }

    if (ui->checkBox_autoscan->isChecked() != ctrl_autoscan)
    {
        if ((*parser).HasMember("ctrl") == false)
        {
            Value contacts(kObjectType);
            (*parser).AddMember("ctrl", contacts, parser->GetAllocator());
        }
        if ((*parser)["ctrl"].HasMember("autoscan") == false)
        {
            (*parser)["ctrl"].AddMember("autoscan", ctrl_autoscan, parser->GetAllocator());
        }

        ctrl_autoscan = ui->checkBox_autoscan->isChecked();
        (*parser)["ctrl"]["autoscan"].SetBool(ctrl_autoscan);
        write++;
    }

    if (ui->checkBox_locks->isChecked() != ctrl_locks)
    {
        if ((*parser).HasMember("ctrl") == false)
        {
            Value contacts(kObjectType);
            (*parser).AddMember("ctrl", contacts, parser->GetAllocator());
        }
        if ((*parser)["ctrl"].HasMember("locks") == false)
        {
            (*parser)["ctrl"].AddMember("locks", ctrl_locks, parser->GetAllocator());
        }

        ctrl_locks = ui->checkBox_locks->isChecked();
        (*parser)["ctrl"]["locks"].SetBool(ctrl_locks);
        write++;
    }

    if (ui->spinBox_freq->value() != ctrl_freq)
    {
        if ((*parser).HasMember("ctrl") == false)
        {
            Value contacts(kObjectType);
            (*parser).AddMember("ctrl", contacts, parser->GetAllocator());
        }
        if ((*parser)["ctrl"].HasMember("freq") == false)
        {
            (*parser)["ctrl"].AddMember("freq", ctrl_freq, parser->GetAllocator());
        }

        ctrl_freq = ui->spinBox_freq->value();
        (*parser)["ctrl"]["freq"].SetInt(ctrl_freq);
        write++;
    }

    // Write modified settings to file
    if (write > 0)
    {
        writeSettings();
    }

    // Close setting window
    close();
}

bool Settings::getAutoScan()
{
    return ctrl_autoscan;
}

bool Settings::getLock()
{
    return ctrl_locks;
}

bool Settings::getPause()
{
    return ui_pause;
}

int Settings::getFreq()
{
    return ctrl_freq;
}

std::vector <struct portconfig_s> Settings::getSerialPortsConfig()
{
    return serial_ports;
}

int Settings::readSettings()
{
    int retcode = 0;
    //std::cout << "readSettings()" << std::endl;

    FILE *fp = std::fopen(filepath.c_str(), "r");

    // Parse from file, if it exists
    if (fp == nullptr)
    {
        std::cerr << "Warning: no configuration file, using default values! A new one will be created next time you change a setting." << std::endl;
    }
    else
    {
        char readBuffer[8192];
        FileReadStream is(fp, readBuffer, sizeof(readBuffer));

        // Try document parsing
        if (parser->ParseStream(is).HasParseError() == true)
        {
            std::cerr << "Parsing settings from file to document failed!" << std::endl;
        }
        else
        {
            retcode = 1;
        }

        std::fclose(fp);
    }

    // Parse from string (fallback)
    if (retcode == 0)
    {
        std::string fallback_json = "{\"ui\":{\"pause\":false},\"ctrl\":{\"autoscan\":true,\"locks\":true,\"freq\":10},\"serial\":{\"ports\":[{\"on\":false,\"path\":\"/dev/ttyUSB0\",\"protocol\":\"auto\",\"speed\":1000000},{\"on\":false,\"path\":\"/dev/ttyUSB1\",\"protocol\":\"auto\",\"speed\":1000000},{\"on\":false,\"path\":\"/dev/ttyACM0\",\"protocol\":\"auto\",\"speed\":1000000},{\"on\":false,\"path\":\"null\",\"protocol\":\"auto\",\"speed\":1000000}]}}";

        if (parser->Parse(fallback_json.c_str()).HasParseError() == true)
        {
            std::cerr << "Parsing settings from fallback string to document failed!" << std::endl;
        }
        else
        {
            std::cout << "Parsing settings from fallback string to document succeeded." << std::endl;
            retcode = 1;
        }
    }

    if (retcode == 1)
    {
        if (parser->HasMember("ui"))
        {
            if ((*parser)["ui"].HasMember("pause"))
            {
                ui_pause = (*parser)["ui"]["pause"].GetBool();
            }
        }

        if (parser->HasMember("ctrl"))
        {
            if ((*parser)["ctrl"].HasMember("autoscan"))
            {
                ctrl_autoscan = (*parser)["ctrl"]["autoscan"].GetBool();
            }
            if ((*parser)["ctrl"].HasMember("locks"))
            {
                ctrl_locks = (*parser)["ctrl"]["locks"].GetBool();
            }
            if ((*parser)["ctrl"].HasMember("freq"))
            {
                int ctrl_freq_temp = (*parser)["ctrl"]["freq"].GetInt();
                if (ctrl_freq_temp > 0 && ctrl_freq_temp < 121)
                {
                    ctrl_freq = ctrl_freq_temp;
                }
            }
        }

        if (parser->HasMember("serial"))
        {
            if ((*parser)["serial"].HasMember("ports"))
            {
                const Value &p = (*parser)["serial"]["ports"];

                if (p.IsArray())
                {
                    serial_ports.clear();

                    for (SizeType i = 0; i < p.Size(); i++)
                    {
                        struct portconfig_s port;
                        port.on = p[i]["on"].GetBool();
                        port.path = p[i]["path"].GetString();
                        port.protocol = p[i]["protocol"].GetString();
                        port.speed = p[i]["speed"].GetInt();
                        serial_ports.push_back(port);
                    }
                }
            }
        }
    }

    return retcode;
}

int Settings::writeSettings()
{
    int retcode = 0;
    //std::cout << "writeSettings() " << std::endl;

    if (filepath.empty() == false)
    {
        FILE *fp = std::fopen(filepath.c_str(), "w");

        if (fp != nullptr)
        {
            char writeBuffer[8192];
            FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

            if (parser->IsObject() == false)
            {
                std::cerr << "Writing settings to '" << filepath << "' failed!" << std::endl;
            }
            else
            {
                PrettyWriter<FileWriteStream> writer(os);
                parser->Accept(writer);

                retcode = 1;
            }

            std::fclose(fp);
        }

    }
    else
    {
        std::cerr << "Writing settings to disk failed: no path for the setting file!" << std::endl;
    }

    return retcode;
}
