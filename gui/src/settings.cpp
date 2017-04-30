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

#include <cstdio>
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
    delete parser;
}

void Settings::loadSettings()
{
    // settings
    ui->checkBox_pause->setChecked(ui_pause);
    ui->checkBox_autoscan->setChecked(ctrl_autoscan);
    ui->checkBox_locks->setChecked(ctrl_locks);
    ui->spinBox_freq->setValue(ctrl_freq);

    // serial ports
    if (serial_ports.size() > 0)
    {
        ui->checkBox_0->setChecked(serial_ports.at(0).on);
        ui->lineEdit_0->setText(QString::fromStdString(serial_ports.at(0).path));
        if (serial_ports.at(0).protocol)
        ui->comboBox_0->setCurrentIndex(serial_ports.at(0).protocol);
        ui->spinBox_0->setValue(serial_ports.at(0).speed);
    }
    if (serial_ports.size() > 1)
    {
        ui->checkBox_1->setChecked(serial_ports.at(1).on);
        ui->lineEdit_1->setText(QString::fromStdString(serial_ports.at(1).path));
        if (serial_ports.at(1).protocol)
        ui->comboBox_1->setCurrentIndex(serial_ports.at(1).protocol);
        ui->spinBox_1->setValue(serial_ports.at(1).speed);
    }
    if (serial_ports.size() > 2)
    {
        ui->checkBox_2->setChecked(serial_ports.at(2).on);
        ui->lineEdit_2->setText(QString::fromStdString(serial_ports.at(2).path));
        if (serial_ports.at(2).protocol)
        ui->comboBox_2->setCurrentIndex(serial_ports.at(2).protocol);
        ui->spinBox_2->setValue(serial_ports.at(2).speed);
    }
    if (serial_ports.size() > 3)
    {
        ui->checkBox_3->setChecked(serial_ports.at(3).on);
        ui->lineEdit_3->setText(QString::fromStdString(serial_ports.at(3).path));
        if (serial_ports.at(3).protocol)
        ui->comboBox_3->setCurrentIndex(serial_ports.at(3).protocol);
        ui->spinBox_3->setValue(serial_ports.at(3).speed);
    }
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

    // Serial ports
    if (parser->HasMember("serial") == false)
    {
        Value contacts(kObjectType);
        (*parser).AddMember("serial", contacts, parser->GetAllocator());
    }
    // Read array from GUI
    if (serial_ports.empty())
    {
        portConfig a, b, c ,d;
        a.on = ui->checkBox_0->isChecked();
        b.on = ui->checkBox_1->isChecked();
        c.on = ui->checkBox_2->isChecked();
        d.on = ui->checkBox_3->isChecked();
        a.path = ui->lineEdit_0->text().toStdString();
        b.path = ui->lineEdit_1->text().toStdString();
        c.path = ui->lineEdit_2->text().toStdString();
        d.path = ui->lineEdit_3->text().toStdString();
        a.protocol = ui->comboBox_0->currentIndex();
        b.protocol = ui->comboBox_1->currentIndex();
        c.protocol = ui->comboBox_2->currentIndex();
        d.protocol = ui->comboBox_3->currentIndex();
        a.speed = ui->spinBox_0->value();
        b.speed = ui->spinBox_1->value();
        c.speed = ui->spinBox_2->value();
        d.speed = ui->spinBox_3->value();
        serial_ports.push_back(a);
        serial_ports.push_back(b);
        serial_ports.push_back(c);
        serial_ports.push_back(d);
    }
    else
    {
        if (serial_ports.at(0).on != ui->checkBox_0->isChecked()) write++;
        serial_ports.at(0).on = ui->checkBox_0->isChecked();
        if (serial_ports.at(1).on != ui->checkBox_1->isChecked()) write++;
        serial_ports.at(1).on = ui->checkBox_1->isChecked();
        if (serial_ports.at(2).on != ui->checkBox_2->isChecked()) write++;
        serial_ports.at(2).on = ui->checkBox_2->isChecked();
        if (serial_ports.at(3).on != ui->checkBox_3->isChecked()) write++;
        serial_ports.at(3).on = ui->checkBox_3->isChecked();

        if (serial_ports.at(0).path != ui->lineEdit_0->text().toStdString()) write++;
        serial_ports.at(0).path = ui->lineEdit_0->text().toStdString();
        if (serial_ports.at(1).path != ui->lineEdit_1->text().toStdString()) write++;
        serial_ports.at(1).path = ui->lineEdit_1->text().toStdString();
        if (serial_ports.at(2).path != ui->lineEdit_2->text().toStdString()) write++;
        serial_ports.at(2).path = ui->lineEdit_2->text().toStdString();
        if (serial_ports.at(3).path != ui->lineEdit_3->text().toStdString()) write++;
        serial_ports.at(3).path = ui->lineEdit_3->text().toStdString();

        if (serial_ports.at(0).protocol != ui->comboBox_0->currentIndex()) write++;
        serial_ports.at(0).protocol = ui->comboBox_0->currentIndex();
        if (serial_ports.at(1).protocol != ui->comboBox_1->currentIndex()) write++;
        serial_ports.at(1).protocol = ui->comboBox_1->currentIndex();
        if (serial_ports.at(2).protocol != ui->comboBox_2->currentIndex()) write++;
        serial_ports.at(2).protocol = ui->comboBox_2->currentIndex();
        if (serial_ports.at(3).protocol != ui->comboBox_3->currentIndex()) write++;
        serial_ports.at(3).protocol = ui->comboBox_3->currentIndex();

        if (serial_ports.at(0).speed != ui->spinBox_0->value()) write++;
        serial_ports.at(0).speed = ui->spinBox_0->value();
        if (serial_ports.at(1).speed != ui->spinBox_1->value()) write++;
        serial_ports.at(1).speed = ui->spinBox_1->value();
        if (serial_ports.at(2).speed != ui->spinBox_2->value()) write++;
        serial_ports.at(2).speed = ui->spinBox_2->value();
        if (serial_ports.at(3).speed != ui->spinBox_3->value()) write++;
        serial_ports.at(3).speed = ui->spinBox_3->value();
    }

    if ((*parser)["serial"].HasMember("ports") == false)
    {
        // to json
        Value port_array(kArrayType);
        for (unsigned i = 0; i < serial_ports.size(); i++)
        {
            Value port(kObjectType);
            port.AddMember("on", serial_ports.at(i).on, parser->GetAllocator());
            Value n(serial_ports.at(i).path.c_str(), parser->GetAllocator());
            port.AddMember("path", n, parser->GetAllocator());
            port.AddMember("protocol", serial_ports.at(i).protocol, parser->GetAllocator());
            port.AddMember("speed", serial_ports.at(i).speed, parser->GetAllocator());
            port_array.PushBack(port, parser->GetAllocator());   // allocator is needed for potential realloc().
        }

        (*parser)["serial"].AddMember("ports", port_array, parser->GetAllocator());
        write++;
    }
    else if (write > 0)
    {
        const Value &p = (*parser)["serial"]["ports"];

        if (p.IsArray())
        {
            for (unsigned i = 0; i < serial_ports.size() && i < p.Size(); i++)
            {
                (*parser)["serial"]["ports"][i]["on"].SetBool(serial_ports.at(i).on);
                (*parser)["serial"]["ports"][i]["path"].SetString(serial_ports.at(i).path.c_str(), parser->GetAllocator());
                (*parser)["serial"]["ports"][i]["protocol"].SetInt(serial_ports.at(i).protocol);
                (*parser)["serial"]["ports"][i]["speed"].SetInt(serial_ports.at(i).speed);
            }
        }
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

const std::vector<portConfig> & Settings::getSerialPortsConfig()
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
        std::string fallback_json = "{\"ui\":{\"pause\":false},\"ctrl\":{\"autoscan\":true,\"locks\":true,\"freq\":10},\"serial\":{\"ports\":[{\"on\":false,\"path\":\"/dev/ttyUSB0\",\"protocol\":0,\"speed\":1000000},{\"on\":false,\"path\":\"/dev/ttyACM0\",\"protocol\":1,\"speed\":1000000},{\"on\":false,\"path\":\"/dev/cu.usbserial\",\"protocol\":2,\"speed\":1000000},{\"on\":false,\"path\":\"/dev/ttyS1\",\"protocol\":2,\"speed\":1000000}]}}";

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
                        struct portConfig port;
                        port.on = p[i]["on"].GetBool();
                        port.path = p[i]["path"].GetString();
                        if (p[i]["protocol"].IsInt()) // compatibility: this used to be a string
                            port.protocol = p[i]["protocol"].GetInt();
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
