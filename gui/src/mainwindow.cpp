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
 * \file mainwindow.cpp
 * \date 27/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

// UI
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qabout.h"

// SmartServoFramework
#include "../../src/DynamixelController.h"
#include "../../src/HerkuleXController.h"
#include "../../src/DynamixelTools.h"
#include "../../src/HerkuleXTools.h"

// Qt
#include <QFile>
#include <QTimer>
#include <QApplication>
#include <QLabel>
#include <QComboBox>
#include <QTextStream>
#include <QTextBrowser>
#include <QMessageBox>
#include <QSignalMapper>
#include <QCloseEvent>

// C++ standard libraries
#include <iostream>

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // UI
    ui->setupUi(this);
    ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->deviceTreeWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    ui->serialPortErrors_label->hide();
    ui->frame_err_dxl1->hide();
    ui->frame_err_dxl2->hide();
    ui->frame_err_hkx->hide();
    ui->frameStatus->hide();

    // Force custom window size
    setGeometry(0, 0, 1200, 640);

    // Save "loading" tab widget, it's the 5th (and last) tab
    loadingTabWidget = ui->tabWidget->widget(ui->tabWidget->count() - 1);

    // QTreeWidget contextual actions
    ui->deviceTreeWidget->setContextMenuPolicy(Qt::ActionsContextMenu);
    refreshAction = new QAction(tr("&Refresh"), this);
    rebootAction = new QAction(tr("Reb&oot"), this);
    resetAction = new QAction(tr("Re&set"), this);
    connect(refreshAction, SIGNAL(triggered()), this, SLOT(refreshServo()));
    connect(rebootAction, SIGNAL(triggered()), this, SLOT(rebootServo()));
    connect(resetAction, SIGNAL(triggered()), this, SLOT(resetServo()));

    // Connect various UI slots
    QObject::connect(ui->actionAdvance_Scanner, SIGNAL(triggered()), this, SLOT(advanceScannerStart()));
    QObject::connect(ui->actionSettings, SIGNAL(triggered()), this, SLOT(settingsStart()));
    QObject::connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    QObject::connect(ui->actionAboutQt, SIGNAL(triggered()), this, SLOT(aboutQt()));
    QObject::connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));

    //QObject::connect(ui->pushButton_updateStatus, SIGNAL(clicked()), this, SLOT(clearErrors()));
    QObject::connect(ui->pushButton_clearStatus, SIGNAL(clicked()), this, SLOT(clearErrors()));

    QObject::connect(ui->pushScanSerial, SIGNAL(clicked()), this, SLOT(scanSerialPorts()));
    QObject::connect(ui->pushScanServo, SIGNAL(clicked()), this, SLOT(scanServos()));
    QObject::connect(ui->deviceTreeWidget, SIGNAL(itemSelectionChanged()), this, SLOT(servoSelection()));

    // Resize content when changing tab or selecting servo
    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(resizeTabWidgetContent()));

    // Initialize device "self refresh" loop
    selfRefreshTimer = new QTimer(this);
    connect(selfRefreshTimer, SIGNAL(timeout()), this, SLOT(servoUpdate()));

    // Advance Scanner window
    asw = NULL;

    // Setting window
    stw = new Settings();
    stw->readSettings();
    stw->loadSettings();

    tableServoSerie = 0;
    tableServoModel = 0;
    tableAutoSelection = false;
}

MainWindow::~MainWindow()
{
    if (asw)
    {
        asw->close();
        delete asw;
        asw = NULL;
    }

    if (stw)
    {
        stw->close();
        delete stw;
        stw = NULL;
    }

    delete ui;
    delete selfRefreshTimer;

    // Clean up controllers
    for (auto p: serialPorts)
    {
        if (p != NULL)
        {
            if (p->deviceController != NULL)
            {
                delete p->deviceController;
                p->deviceController = NULL;
            }

            delete p;
            p = NULL;
        }
    }
}

void MainWindow::loadingScreen(bool enabled)
{
    QTabBar *tabBar = ui->tabWidget->findChild<QTabBar *>();
    tabBar->setVisible(!enabled);
    ui->tabWidget->setDocumentMode(enabled);

    if (enabled == true)
    {
        ui->frame_loading->show();

        if (ui->tabWidget->count() != 5)
        {
            ui->tabWidget->addTab(loadingTabWidget, "loading");
        }
        ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);

        // Show "help" panel while loading GUI
        int x = ui->label_loading_img->size().width();
        int y = ui->label_loading_img->size().height();
        ui->label_loading_img->setMaximumWidth(x);
        ui->label_loading_img->setMaximumHeight(y);

        if ((rand() % 1024) >= 512)
        {
            QPixmap load(":/help/help/Dynamixel_help.png");
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
            load.setDevicePixelRatio(qApp->devicePixelRatio());
#endif
            ui->label_loading_img->setPixmap(load);
        }
        else
        {
            QPixmap load(":/help/help/HerkuleX_help.png");
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
            load.setDevicePixelRatio(qApp->devicePixelRatio());
#endif
            ui->label_loading_img->setPixmap(load);
        }
    }
    else
    {
        ui->frame_loading->hide();

        ui->tabWidget->removeTab(4);
        ui->tabWidget->setCurrentIndex(0);
    }
}

void MainWindow::autoScan()
{
    bool autoscan = true;

    if (stw)
    {
        autoscan = stw->getAutoScan();
    }

    scanSerialPorts(autoscan);
}

void MainWindow::scanSerialPorts(bool autoscan)
{
    // Disable scan buttons, indicate we are starting to scan
    ui->pushScanSerial->setDisabled(true);
    ui->pushScanServo->setDisabled(true);
    loadingScreen(true);

    // Clean the deviceTreeWidget content
    while (QWidget *item = ui->deviceTreeWidget->childAt(0,0))
    {
        delete item;
    }
    ui->deviceTreeWidget->clear();

    // Clean the "scan group box" widgets
    while (QLayoutItem *item = ui->groupPorts->layout()->takeAt(0))
    {
        if (QWidget *widget = item->widget())
        {
            delete widget;
        }
        delete item;
    }
    ui->groupPorts->update();

    // Clean existing serial ports list and associated controllers
    for (auto p: serialPorts)
    {
        if (p != NULL)
        {
            if (p->deviceController != NULL)
            {
                delete p->deviceController;
                p->deviceController = NULL;
            }

            delete p;
            p = NULL;
        }
    }
    serialPorts.clear();

    // Scan for available/new serial ports
    std::vector <std::string> availablePorts;
    serialPortsScanner(availablePorts);

    if (availablePorts.size() == 0)
    {
        ui->groupPorts->hide();
        ui->serialPortErrors_label->show();

        QTreeWidgetItem *port = new QTreeWidgetItem();
        port->setText(0, tr("No serial port available!"));
        ui->deviceTreeWidget->addTopLevelItem(port);
    }
    else
    {
        ui->groupPorts->show();
        ui->serialPortErrors_label->hide();

        QSignalMapper *signalMapper = new QSignalMapper(this);
        QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(refreshSerialPort(QString)));

        // Create "helper" structure to keep track of serial port instances
        // Create an entry inside the "scan group box" for each serial port
        for (auto p: availablePorts)
        {
            SerialPortHelper *helper = new SerialPortHelper();
            serialPorts.push_back(helper);

            helper->deviceName = new QCheckBox(QString::fromStdString(p));
            helper->deviceName->setChecked(true);
            helper->deviceSettings = new QComboBox();
            helper->deviceSettings->addItem("auto");
            helper->deviceSettings->addItem("DXL v1 @ 1 Mb/s");
            helper->deviceSettings->addItem("DXL v1 @ 115.2 Kb/s");
            helper->deviceSettings->addItem("DXL v1 @ 57.6 Kb/s");
            helper->deviceSettings->addItem("DXL v2 @ 1 Mb/s");
            helper->deviceSettings->addItem("DXL v2 @ 115.2 Kb/s");
            helper->deviceSettings->addItem("DXL v2 @ 57.6 Kb/s");
            helper->deviceSettings->addItem("HKX @ 115.2 Kb/s");
            helper->deviceSettings->addItem("HKX @ 57.6 Kb/s");
            helper->deviceSettings->addItem("HKX @ 1 Mb/s");
            helper->deviceSettings->setMaximumSize(80, 28);
            helper->deviceSettings->setFont(QFont("Cantarell", 10));
            helper->deviceScan = new QPushButton();
            helper->deviceScan->setIcon(QIcon(":/icons/icons/emblem-ubuntuone-updating.svg"));
            helper->deviceScan->setIconSize(QSize(24, 24));
            helper->deviceScan->setMaximumSize(24, 24);
            helper->deviceScan->setFlat(true);
            ui->groupPorts->layout()->addWidget(helper->deviceName);
            ui->groupPorts->layout()->addWidget(helper->deviceSettings);
            ui->groupPorts->layout()->addWidget(helper->deviceScan);
            ui->groupPorts->update();

            signalMapper->setMapping(helper->deviceScan, QString(helper->deviceName->text()));
            QObject::connect(helper->deviceScan, SIGNAL(clicked()), signalMapper, SLOT(map()));
        }

        // Update the GUI before each servo scan
        qApp->processEvents();

        // Launch servo scanning?
        if (autoscan == true)
        {
            scanServos();
        }
    }

    // Indicate that we are no longer scanning
    ui->pushScanSerial->setEnabled(true);
    ui->pushScanServo->setEnabled(true);

    // Maybe this scan did not yield any results, to avoid showing a blank interface,
    // we do not turn off loading screen, only the loading animation
    ui->frame_loading->hide();

    // Update UI
    ui->deviceTreeWidget->update();
}

void MainWindow::scanServos()
{
    ui->pushScanSerial->setDisabled(true);
    ui->pushScanServo->setDisabled(true);
    tableAutoSelection = false;
    scan_running = true;

    for (SerialPortHelper *h: serialPorts)
    {
        if (scan_running == true)
        {
            // Process the device only if its GUI element is 'checked'
            if (h->deviceName->isChecked() == true)
            {
                // Launch a scan
                scanServos(h->deviceName->text());
            }
        }
    }

    scan_running = false;

    ui->pushScanSerial->setEnabled(true);
    ui->pushScanServo->setEnabled(true);
}

void MainWindow::refreshSerialPort(QString port_qstring)
{
    ui->pushScanSerial->setDisabled(true);
    ui->pushScanServo->setDisabled(true);
    tableAutoSelection = false;
    scan_running = true;

    // Launch a scan
    scanServos(port_qstring);

    scan_running = false;

    ui->pushScanSerial->setEnabled(true);
    ui->pushScanServo->setEnabled(true);
}

void MainWindow::scanServos(QString port_qstring)
{
    // Disable scan buttons
    ui->pushScanSerial->setDisabled(true);
    ui->pushScanServo->setDisabled(true);

    bool ctrl_locks = stw->getLock();
    int ctrl_freq = stw->getFreq();

    // First locate the port to scan
    for (struct SerialPortHelper *h: serialPorts)
    {
        if (h->deviceName->text() == port_qstring)
        {
            std::string port_stdstring = port_qstring.toStdString();
            std::cout << ">> scanServos(" << port_stdstring << ")" << std::endl;

            // Disable the refresh button on this port
            h->deviceScan->setDisabled(true);

            // Indicate we are starting to scan
            // FIXME check if we are on the loading tab
            {
                ui->frame_loading->show();

                ui->tabWidget->update();
                ui->tabWidget->repaint();
                qApp->processEvents();

                // Lock windows size
                this->setMinimumSize(this->size());
                this->setMaximumSize(this->size());
                ui->centralWidget->setMinimumSize(this->size());
                ui->centralWidget->setMaximumSize(this->size());
            }

            // Clean devices attached to the corresponding deviceTreeWidget port (if needed)
            QTreeWidgetItem *port = NULL;
            for (int i = 0; i < ui->deviceTreeWidget->topLevelItemCount(); i++)
            {
                if (ui->deviceTreeWidget->topLevelItem(i)->text(0) == h->deviceName->text())
                {
                    port = ui->deviceTreeWidget->topLevelItem(i);
                    while (port->childCount() > 0)
                    {
                        QTreeWidgetItem *child = ui->deviceTreeWidget->topLevelItem(i)->child(0);
                        port->removeChild(child);
                        delete child;
                    }
                }
            }

            // Add port in deviceTreeWidget (if needed)
            if (port == NULL)
            {
                port = new QTreeWidgetItem();
                ui->deviceTreeWidget->addTopLevelItem(port);
                port->setText(0, port_qstring);
            }

            // Indicate we are scanning this port in the device tree widget
            QString scan_entry_txt = tr("Scanning...");
            QTreeWidgetItem *scan_entry = new QTreeWidgetItem();
            scan_entry->setText(0, scan_entry_txt);
            port->setExpanded(true);
            port->addChild(scan_entry);

            // Handle "auto" mode
            int scan_rounds  = 1;
            int scan_results = 0;
            int scan_setting = 0;

            // Handle "saved" settings
            int deviceControllerProtocolSAVED = 0;
            int deviceControllerSpeedSAVED = 0;

            for (int i = 0; i < scan_rounds; i++)
            {
                if (h->deviceSettings->currentIndex() == 0)
                {
                    // Rewrite 'scan_setting' if scan preset is set to 'auto'
                    int scanrounds_item[4] = {1, 3, 4, 7};
                    scan_rounds = 4;
                    scan_results = 0;
                    scan_setting = scanrounds_item[i];
                }
                else
                {
                    scan_rounds  = 1;
                    scan_results = 0;
                    scan_setting = h->deviceSettings->currentIndex();

                    // We need to check is the protocol or speed has changed on the selected port
                    deviceControllerProtocolSAVED = h->deviceControllerProtocol;
                    deviceControllerSpeedSAVED = h->deviceControllerSpeed;
                }

                // Load controller settings
                switch (scan_setting)
                {
                // DXL v1
                case 1:
                    h->deviceControllerProtocol = 1;
                    h->deviceControllerSpeed = 1000000;
                    h->deviceControllerDevices = SERVO_MX;
                    break;
                case 2:
                    h->deviceControllerProtocol = 1;
                    h->deviceControllerSpeed = 115200;
                    h->deviceControllerDevices = SERVO_MX;
                    break;
                case 3:
                    h->deviceControllerProtocol = 1;
                    h->deviceControllerSpeed = 57600;
                    h->deviceControllerDevices = SERVO_MX;
                    break;

                // DXL v2
                case 4:
                    h->deviceControllerProtocol = 2;
                    h->deviceControllerSpeed = 1000000;
                    h->deviceControllerDevices = SERVO_XL;
                    break;
                case 5:
                    h->deviceControllerProtocol = 2;
                    h->deviceControllerSpeed = 115200;
                    h->deviceControllerDevices = SERVO_XL;
                    break;
                case 6:
                    h->deviceControllerProtocol = 2;
                    h->deviceControllerSpeed = 57600;
                    h->deviceControllerDevices = SERVO_XL;
                    break;

                // HKX
                case 7:
                    h->deviceControllerProtocol = 3;
                    h->deviceControllerSpeed = 115200;
                    h->deviceControllerDevices = SERVO_DRS;
                    break;
                case 8:
                    h->deviceControllerProtocol = 3;
                    h->deviceControllerSpeed = 57600;
                    h->deviceControllerDevices = SERVO_DRS;
                    break;
                case 9:
                    h->deviceControllerProtocol = 3;
                    h->deviceControllerSpeed = 1000000;
                    h->deviceControllerDevices = SERVO_DRS;
                    break;
                }

                // Do we need a new controller for this serial port?
                // - no controller instanciated
                // - controller with new protocol or speed
                if (h->deviceController == NULL ||
                    deviceControllerProtocolSAVED != h->deviceControllerProtocol ||
                    deviceControllerSpeedSAVED != h->deviceControllerSpeed)
                {
                    // Delete old controller (if needed)
                    if (h->deviceController != NULL)
                    {
                        h->deviceController->disconnect();
                        delete h->deviceController;
                        h->deviceController = NULL;
                    }

                    // Create a new one
                    if (h->deviceControllerProtocol == 1)
                    {
                        h->deviceController = new DynamixelController(ctrl_freq, h->deviceControllerDevices);
                    }
                    else if (h->deviceControllerProtocol == 2)
                    {
                        h->deviceController = new DynamixelController(ctrl_freq, h->deviceControllerDevices);
                    }
                    else if (h->deviceControllerProtocol == 3)
                    {
                        h->deviceController = new HerkuleXController(ctrl_freq, h->deviceControllerDevices);
                    }

                    // Connect the controller to its serial port
                    if (h->deviceController != NULL)
                    {
                        scan_results = h->deviceController->connect(port_stdstring, h->deviceControllerSpeed);

                        if (scan_results != 1)
                        {
                            h->deviceController->disconnect();
                            delete h->deviceController;
                            h->deviceController = NULL;
                        }
                    }
                }

                // Scan
                if (scan_running == true && h->deviceController != NULL)
                {
                    loadingScreen(true);
/*
                    // Are we scanning the currently selected port? Then go to the loading screen
                    ControllerAPI *ctrl = NULL;
                    getCurrentController(ctrl);

                    if (h->deviceController == ctrl)
                    {
                        std::cout << "YES WE ARE" << std::endl;
                        loadingScreen(true);
                        qApp->processEvents();
                    }
*/
                    // Scan for servo(s)
                    h->deviceController->autodetect(ui->rangeStart_spinBox->value(), ui->rangeStop_spinBox->value());

                    // Controller already in ready state? Wait for the new scan to begin then
                    if (h->deviceController->getState() >= state_scanned)
                    {
                        if (scan_running == false)
                        {
                            h->deviceController->disconnect();
                            break;
                        }

                        while (h->deviceController->getState() > state_scanned)
                        {
                            std::chrono::milliseconds waittime(static_cast<int>(3));
                            std::this_thread::sleep_for(waittime);
                        }
                    }

                    // Wait until the controller is 'scanned' (not 'ready', cause if there is no results it will never be ready)
                    while (h->deviceController->getState() >= state_started &&
                           h->deviceController->getState() < state_scanned)
                    {
                        if (scan_running == false)
                        {
                            h->deviceController->disconnect();
                            break;
                        }

                        std::chrono::milliseconds waittime(static_cast<int>(3));
                        std::this_thread::sleep_for(waittime);
                        qApp->processEvents();
                    }

                    // Get the scanned list
                    const std::vector <Servo *> servos = h->deviceController->getServos();

                    if (servos.size() != 0)
                    {
                        // Add device(s) found to the tree widget
                        for (auto s: servos)
                        {
                            scan_results++;

                            // Add it to the device tree widget
                            QString device_txt = "[#" + QString::number(s->getId()) + "]  " + QString::fromStdString(s->getModelString());
                            QTreeWidgetItem *device = new QTreeWidgetItem();
                            port->addChild(device);
                            if (h->deviceControllerProtocol == 3)
                            {
                                device->setIcon(0, QIcon(":/devices/devices/HKX.svg"));
                            }
                            else
                            {
                                if (h->deviceControllerProtocol == 2)
                                    device->setIcon(0, QIcon(":/devices/devices/DXL.svg")); // DXLv2.svg
                                else if (h->deviceControllerProtocol == 1)
                                    device->setIcon(0, QIcon(":/devices/devices/DXL.svg")); // DXLv1.svg
                            }
                            device->setText(0, device_txt);

                            if (tableAutoSelection == false)
                            {
                                // Autoselect first servo
                                ui->deviceTreeWidget->clearSelection();
                                device->setSelected(true);
                                tableAutoSelection = true;

                                loadingScreen(false);
                                selfRefreshTimer->start(125);
                            }
                        }

                        // Wait until the controller is ready
                        while (h->deviceController->getState() >= state_started &&
                               h->deviceController->getState() < state_ready)
                        {
                            std::chrono::milliseconds waittime(static_cast<int>(4));
                            std::this_thread::sleep_for(waittime);
                            qApp->processEvents();
                        }

                        // Exit 'scan_rounds'
                        break;
                    }
                    else
                    {
                        h->deviceController->disconnect();
                    }
                }
                else
                {
                    std::cerr << "Error, cannot start scanning for servos! (scann running ? " << scan_running << ")   (controller ? " << h->deviceController << ")" << std::endl;
                }
            }

            if (scan_running == false)
            {
                h->deviceController->disconnect();
            }

            // Indicate that we are no longer scanning
            ui->frame_loading->hide();
            port->removeChild(scan_entry);
            delete scan_entry;

            // Do that only once, not every time we try a new serial port configuration
            if (scan_results <= 0)
            {
                if (scan_results == 0)
                {
                    // Indicate we did not found any device
                    QTreeWidgetItem *nodevice = new QTreeWidgetItem();
                    nodevice->setText(0, tr("No device available!"));
                    port->addChild(nodevice);
                }
                else if (scan_results == -1)
                {
                    // Put a 'locked' icon
                    QString achtung_txt = "Interface is locked!";
                    QTreeWidgetItem *achtung = new QTreeWidgetItem();
                    port->addChild(achtung);
                    achtung->setText(0, achtung_txt);
                    achtung->setIcon(0, QIcon(":/icons/icons/emblem-readonly.svg"));
                }
                else if (scan_results == -2)
                {
                    // Put an 'error' icon
                    QString achtung_txt = "Unable to connect!";
                    QTreeWidgetItem *achtung = new QTreeWidgetItem();
                    port->addChild(achtung);
                    achtung->setText(0, achtung_txt);
                    achtung->setIcon(0, QIcon(":/icons/icons/emblem-unreadable.svg"));
                }

                // No need to keep this "empty" controller working
                delete h->deviceController;
                h->deviceController = NULL;
            }

            // Unlock windows size
            this->setMinimumSize(0, 0);
            this->setMaximumSize(4096, 4096);
            ui->centralWidget->setMinimumSize(0, 0);
            ui->centralWidget->setMaximumSize(4096, 4096);

            h->deviceScan->setEnabled(true);

            // Refresh UI
            ui->deviceTreeWidget->update();
        }
    }

    ui->pushScanSerial->setEnabled(true);
    ui->pushScanServo->setEnabled(true);
}

int MainWindow::getCurrentController(ControllerAPI *&ctrl)
{
    int retcode = 0;

    // Get selected item
    if (ui->deviceTreeWidget->selectedItems().size() > 0)
    {
        QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

        // Check if the item exist, plus if this a device and not a port
        if (item != NULL && item->parent() != NULL)
        {
            // Print status?
            std::string port = item->parent()->text(0).toStdString();
            //std::cout << "> Selected serial port: " << port << std::endl;

            // Search for the port
            for (auto p: serialPorts)
            {
                // Found it
                if (p->deviceName->text().toStdString() == port)
                {
                    ctrl = p->deviceController;

                    if (ctrl != NULL)
                    {
                        retcode = 1;
                    }
                }
            }
        }
    }

    return retcode;
}

int MainWindow::getCurrentServo(ControllerAPI *&ctrl, int &id)
{
    int retcode = 0;

    // Get selected item
    if (ui->deviceTreeWidget->selectedItems().size() > 0)
    {
        QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

        // Check if the item exist, plus if tis a device and not a port
        if (item != NULL && item->parent() != NULL)
        {
            int sid = 0;

            // Extract servo id (the hard way...)
            if (item->text(0).lastIndexOf("]") == 3)
            {
                sid = item->text(0).midRef(2, 1).toString().toUInt();
            }
            else if (item->text(0).lastIndexOf("]") == 4)
            {
                sid = item->text(0).midRef(2, 2).toString().toUInt();
            }
            else if (item->text(0).lastIndexOf("]") == 5)
            {
                sid = item->text(0).midRef(2, 3).toString().toUInt();
            }

            // Print status?
            std::string port = item->parent()->text(0).toStdString();
            //std::string device = item->text(0).toStdString();
            //std::cout << "> Selected device '" << sid << "' from " << device << " on port: " << port << std::endl;

            // Search for the duo port/servo
            for (auto p: serialPorts)
            {
                // We have the port
                if (p != NULL && p->deviceName->text().toStdString() == port)
                {
                    // We have the servo
                    if (sid >= 0 && sid < 254)
                    {
                        if (p->deviceController != NULL)
                        {
                            ctrl = p->deviceController;
                            id = sid;
                            retcode = 1;
                        }
                    }
                }
            }
        }
    }

    return retcode;
}

int MainWindow::getCurrentServo(Servo *&servo)
{
    int retcode = 0;

    // Get selected item
    if (ui->deviceTreeWidget->selectedItems().size() > 0)
    {
        QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

        // Check if the item exist, plus if this a device and not a port
        if (item != NULL && item->parent() != NULL)
        {
            int sid = 0;

            // Extract servo id (the hard way...)
            if (item->text(0).lastIndexOf("]") == 3)
            {
                sid = item->text(0).midRef(2, 1).toString().toUInt();
            }
            else if (item->text(0).lastIndexOf("]") == 4)
            {
                sid = item->text(0).midRef(2, 2).toString().toUInt();
            }
            else if (item->text(0).lastIndexOf("]") == 5)
            {
                sid = item->text(0).midRef(2, 3).toString().toUInt();
            }

            // Print status?
            std::string port = item->parent()->text(0).toStdString();
            //std::string device = item->text(0).toStdString();
            //std::cout << "> Device '" << sid << "' from " << device << " on port: " << port << std::endl;

            // Search for the duo port/servo
            for (auto p: serialPorts)
            {
                // We found the port
                if (p != NULL && p->deviceName->text().toStdString() == port)
                {
                    // We found the servo
                    if (sid >= 0 && sid < 254)
                    {
                        if (p->deviceController != NULL)
                        {
                            servo = p->deviceController->getServo(sid);
                            if (servo != NULL)
                            {
                                retcode = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    return retcode;
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    resizeTabWidgetContent();
}

void MainWindow::changeEvent(QEvent *event)
{
    if (event->type() == QEvent::WindowStateChange)
    {
        if (stw->getPause() == true)
        {
            //std::cout << "Windows focus changed: pausing controllers threads" << std::endl;

            // Pause controllers
            for (SerialPortHelper *p: serialPorts)
            {
                if (p->deviceController != NULL)
                {
                    p->deviceController->pauseThread();
                }
            }
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    bool ready = true;

    // Check if a controller is not currently busy scanning
    for (auto p: serialPorts)
    {
        if (p->deviceController != NULL)
        {
            if (!(p->deviceController->getState() == state_stopped ||
                  p->deviceController->getState() == state_ready ||
                  p->deviceController->getState() == state_ready))
            {
                ready = false;
                scan_running = false;
            }
        }
    }

    // If a scan is in progress, ignore the close event until the scanning has stopped
    if (ready == false)
    {
        event->ignore();
        scan_running = false;
    }
}

void MainWindow::resizeTabWidgetContent()
{
    if (ui->tabWidget->currentIndex() == 0)
    {
/*
        // FIXME // does not produce expected results
        int tabWidgetSize = ui->tabWidget->size().width();
        ui->frame_quickcontrols->setMaximumWidth(tabWidgetSize * 0.40);
        ui->tableWidget->setMaximumWidth(tabWidgetSize * 0.60);
*/
        int error_margin = 14; // (column count) * (grid line size) + (scroll bar size) ?
        int width_available = ui->tableWidget->size().width();
        int width_header = ui->tableWidget->verticalHeader()->size().width();

        // Scale register table
        if (tableServoSerie >= SERVO_HERKULEX)
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
    else if (ui->tabWidget->currentIndex() == 1)
    {
        // Servo picture
        int w = ui->tabWidget->size().width() * 0.33;
        ui->servoPicture_label->setMinimumWidth(w);
        ui->servoPicture_label->setMinimumHeight(w);
        ui->servoPicture_label->setMaximumWidth(w);
        ui->servoPicture_label->setMaximumHeight(w);

        // TextBrowser will scale itself
    }

    ui->tabWidget->update();
}

void MainWindow::servoSelection()
{
    Servo *servo = NULL;

    // Get currently selected servo
    if (getCurrentServo(servo) > 0)
    {
        toggleServoPanel(true);

        // Contextual menu actions
        ui->deviceTreeWidget->addAction(refreshAction);
        ui->deviceTreeWidget->addAction(rebootAction);
        ui->deviceTreeWidget->addAction(resetAction);

        // Disconnect servo panel slots (will be reconnected on the next "update")
        QObject::disconnect(ui->radioButton_1, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::disconnect(ui->radioButton_2, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::disconnect(ui->torque_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleTorque(bool)));
        QObject::disconnect(ui->led_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleLED(bool)));
        QObject::disconnect(ui->maxTorqueSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMaxTorque(int)));
        QObject::disconnect(ui->torqueLimitSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveTorqueLimit(int)));
        QObject::disconnect(ui->movingSpeedSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMovingSpeed(int)));
        QObject::disconnect(ui->cwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCWLimit(int)));
        QObject::disconnect(ui->ccwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCCWLimit(int)));
        QObject::disconnect(ui->gpos, SIGNAL(valueChanged(int)), this, SLOT(moveServo(int)));
        QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(modifier(int, int)));

        // Adapt to servo model
        ////////////////////////////////////////////////////////////////////////

        int servoSerie, servoModel;
        servo->getModelInfos(servoSerie, servoModel);

        // Clean and generate a new register table (if we switch servo serie)
        // and device error box
        if (servoSerie != tableServoSerie && servoModel != tableServoModel)
        {
            generateRegisterTable(servoSerie, servoModel);

            if (servoSerie >= SERVO_HERKULEX)
            {
                ui->frame_err_dxl1->hide();
                ui->frame_err_dxl2->hide();
                ui->frame_err_hkx->show();

                ui->servoManual_label->setText("<html><head/><body><p><a href=\"http://hovis.co.kr/guide/herkulex_eng.html\"><span style=\" text-decoration: underline; color:#ffffff;\">Download manuals on Dongbu Robot website</span></a></p></body></html>");
                ui->copyrightNotice_label->setText("<html><head/><body><p>Technical specifications provided by <a href=\"www.dongburobot.com\"><span style=\" text-decoration: underline; color:#0000ff;\">dongburobot.com</span></a> website.</p></body></html>");
            }
            else
            {
                ui->frame_err_hkx->hide();

                if (servoSerie >= SERVO_XL)
                {
                    ui->frame_err_dxl1->hide();
                    ui->frame_err_dxl2->show();
                }
                else
                {
                    ui->frame_err_dxl1->show();
                    ui->frame_err_dxl2->hide();
                }

                ui->servoManual_label->setText("<html><head/><body><p><a href=\"http://support.robotis.com/en/\"><span style=\" text-decoration: underline; color:#ffffff;\">Consult the online manuals on Robotis website</span></a></p></body></html>");
                ui->copyrightNotice_label->setText("<html><head/><body><p>Pictures and technical specifications courtesy of <a href=\"www.robotis.com\"><span style=\" text-decoration: underline; color:#0000ff;\">robotis.com</span></a></p></body></html>");
            }
        }
/*
        // Clean and generate a status box (if we swhitch brand)
        if ((servoSerie < SERVO_HERKULEX && tableServoSerie >= SERVO_HERKULEX) ||
            (servoSerie >= SERVO_HERKULEX && tableServoSerie < SERVO_HERKULEX) ||
             ui->groupStatus->layout()->children().isEmpty())
        {
            // Clean
            while (QLayoutItem *item = ui->groupStatus->layout()->takeAt(0))
            {
                if (QWidget *widget = item->widget())
                {
                    delete widget;
                }
                delete item;
            }
            ui->groupStatus->update();

            // Regen
            QWidget *widget = new QWidget;
            if (servoSerie < SERVO_HERKULEX)
            {
                Ui::StatusDXL uistatus;
                uistatus.setupUi(widget);
            }
            else
            {
                Ui::StatusHKX uistatus;
                uistatus.setupUi(widget);
            }

            ui->groupStatus->layout()->addWidget(widget);
            ui->groupStatus->show();
            widget->show();
        }
*/
        // Steps
        ui->cwlimit->setRange(0, servo->getSteps() - 1);
        ui->ccwlimit->setRange(0, servo->getSteps() - 1);
        ui->cpos->setRange(0, servo->getSteps() - 1);
        ui->gpos->setRange(0, servo->getSteps() - 1);

        if (servo->getRunningDegrees() == 360)
        {
            ui->cwlimit->setWrapping(true);
            ui->ccwlimit->setWrapping(true);
            ui->cpos->setWrapping(true);
            ui->gpos->setWrapping(true);
        }
        else
        {
            ui->cwlimit->setWrapping(false);
            ui->ccwlimit->setWrapping(false);
            ui->cpos->setWrapping(false);
            ui->gpos->setWrapping(false);
        }

        // Reset "alarm" colors
        ui->cvolt->setPalette(QPalette(QColor(85,85,128)));
        ui->ctemp->setPalette(QPalette(QColor(85,85,128)));

        // Picture and doc
        QFile servoSpec;
        QPixmap servoIcon;

        int pictureSize = ui->tabWidget->size().height() * 0.33; // All of our pictures are square
        ui->servoPicture_label->setMaximumWidth(pictureSize);
        ui->servoPicture_label->setMaximumHeight(pictureSize);
        ui->servoPicture_label->setScaledContents(true);

        // Model specific
        switch (servoSerie)
        {
        case SERVO_DRS:
            if (servoModel == SERVO_DRS_0101)
            {
                servoIcon.load(":/devices/devices/DRS-0101.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0101.html");
            }
            else if (servoModel == SERVO_DRS_0201)
            {
                servoIcon.load(":/devices/devices/DRS-0201.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0201.html");
            }
            else if (servoModel == SERVO_DRS_0401)
            {
                servoIcon.load(":/devices/devices/DRS-0401.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0401.html");
            }
            else if (servoModel == SERVO_DRS_0402)
            {
                servoIcon.load(":/devices/devices/DRS-0402.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0402.html");
            }
            else if (servoModel == SERVO_DRS_0601)
            {
                servoIcon.load(":/devices/devices/DRS-0601.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0601.html");
            }
            else if (servoModel == SERVO_DRS_0602)
            {
                servoIcon.load(":/devices/devices/DRS-0602.jpg");
                servoSpec.setFileName(":/specs/specs/DRS-0602.html");
            }
            break;

        case SERVO_XL:
            servoIcon.load(":/devices/devices/XL-320.jpg");
            servoSpec.setFileName(":/specs/specs/XL-320.html");
            break;

        case SERVO_EX:
            if (servoSerie == SERVO_EX106)
            {
                servoIcon.load(":/devices/devices/EX-106.jpg");
                servoSpec.setFileName(":/specs/specs/EX-106.html");
            }
            else if (servoSerie == SERVO_EX106p)
            {
                servoIcon.load(":/devices/devices/EX-106p.jpg");
                servoSpec.setFileName(":/specs/specs/EX-106p.html");
            }
            break;

        case SERVO_MX:
            if (servoModel == SERVO_MX106)
            {
                servoIcon.load(":/devices/devices/MX-106.jpg");
                servoSpec.setFileName(":/specs/specs/MX-106.html");
            }
            else if (servoModel == SERVO_MX64)
            {
                servoIcon.load(":/devices/devices/MX-64.jpg");
                servoSpec.setFileName(":/specs/specs/MX-64.html");
            }
            else if (servoModel == SERVO_MX28)
            {
                servoIcon.load(":/devices/devices/MX-28.jpg");
                servoSpec.setFileName(":/specs/specs/MX-28.html");
            }
            else if (servoModel == SERVO_MX12W)
            {
                servoIcon.load(":/devices/devices/MX-28.jpg");
                servoSpec.setFileName(":/specs/specs/MX-12W.html");
            }
            break;

        case SERVO_AX:
        case SERVO_DX:
        case SERVO_RX:
        default: // these series are the "default"
            if (servoSerie == SERVO_DX)
            {
                servoIcon.load(":/devices/devices/DX-117.jpg");
                servoSpec.setFileName(":/specs/specs/DX-117.html");
            }
            else if (servoModel == SERVO_AX12W)
            {
                servoIcon.load(":/devices/devices/AX-12W.jpg");
                servoSpec.setFileName(":/specs/specs/AX-12W.html");
            }
            else if (servoModel == SERVO_AX12A)
            {
                servoIcon.load(":/devices/devices/AX-12A.jpg");
                servoSpec.setFileName(":/specs/specs/AX-12A.html");
            }
            else if (servoModel == SERVO_AX18A)
            {
                servoIcon.load(":/devices/devices/AX-18A.jpg");
                servoSpec.setFileName(":/specs/specs/AX-18A.html");
            }
            else if (servoModel == SERVO_RX24F)
            {
                servoIcon.load(":/devices/devices/RX-24.jpg");
            }
            else if (servoModel == SERVO_RX28)
            {
                servoIcon.load(":/devices/devices/RX-48.jpg");
            }
            else if (servoModel == SERVO_RX64)
            {
                servoIcon.load(":/devices/devices/RX-64.jpg");
            }
            break;
        }

        // Set icon
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
        servoIcon.setDevicePixelRatio(qApp->devicePixelRatio());
#endif
        ui->servoPicture_label->setPixmap(servoIcon);

        // Set specification text
        if (servoSpec.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream flux(&servoSpec);
            ui->textBrowser_spec->setText(flux.readAll());
            servoSpec.close();
        }
        else
        {
            ui->textBrowser_spec->setText(tr("No documentation available..."));
        }

        // Read register values and stuff
        servoUpdate();
    }
    else
    {
        // Clean contextual menu
        ui->deviceTreeWidget->removeAction(refreshAction);
        ui->deviceTreeWidget->removeAction(rebootAction);
        ui->deviceTreeWidget->removeAction(resetAction);

        // Clean servo register panel
        cleanRegisterTable();
        toggleServoPanel(false);
    }

    resizeTabWidgetContent();
}

void MainWindow::servoUpdate()
{
    Servo *servo = NULL;

    // Get selected servo
    if (getCurrentServo(servo) > 0)
    {
        // Disconnect servo panel slots
        QObject::disconnect(ui->radioButton_1, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::disconnect(ui->radioButton_2, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::disconnect(ui->torque_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleTorque(bool)));
        QObject::disconnect(ui->led_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleLED(bool)));
        QObject::disconnect(ui->maxTorqueSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMaxTorque(int)));
        QObject::disconnect(ui->torqueLimitSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveTorqueLimit(int)));
        QObject::disconnect(ui->movingSpeedSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMovingSpeed(int)));
        QObject::disconnect(ui->cwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCWLimit(int)));
        QObject::disconnect(ui->ccwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCCWLimit(int)));
        QObject::disconnect(ui->gpos, SIGNAL(valueChanged(int)), this, SLOT(moveServo(int)));
        QObject::disconnect(ui->gpos, SIGNAL(valueChanged(int)), this, SLOT(moveServo(int)));
        QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(modifier(int, int)));

        // Get servos model infos
        int servoSerie, servoModel;
        servo->getModelInfos(servoSerie, servoModel);

        // Read and set values
        ////////////////////////////////////////////////////////////////////////

        if (servoSerie >= SERVO_HERKULEX)
        {
            updateRegisterTableHerkuleX(servo, servoSerie, servoModel);
        }
        else
        {
            updateRegisterTableDynamixel(servo, servoSerie, servoModel);
        }

        // Error handling
        ////////////////////////////////////////////////////////////////////////

        errorHandling(servo, servoSerie, servoModel);

        // Re-connect servo panel slots
        QObject::connect(ui->radioButton_1, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::connect(ui->radioButton_2, SIGNAL(clicked(bool)), this, SLOT(toggleRunningMode()));
        QObject::connect(ui->torque_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleTorque(bool)));
        QObject::connect(ui->led_checkBox, SIGNAL(clicked(bool)), this, SLOT(toggleLED(bool)));
        QObject::connect(ui->maxTorqueSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMaxTorque(int)));
        QObject::connect(ui->torqueLimitSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveTorqueLimit(int)));
        QObject::connect(ui->movingSpeedSlider, SIGNAL(sliderMoved(int)), this, SLOT(moveMovingSpeed(int)));
        QObject::connect(ui->cwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCWLimit(int)));
        QObject::connect(ui->ccwlimit, SIGNAL(valueChanged(int)), this, SLOT(moveCCWLimit(int)));
        QObject::connect(ui->gpos, SIGNAL(valueChanged(int)), this, SLOT(moveServo(int)));
        QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(modifier(int, int)));
    }
    else
    {
        cleanRegisterTable();
        toggleServoPanel(false);
    }

    // Update
    ui->tableWidget->update();
}

void MainWindow::updateRegisterTableHerkuleX(Servo *servo_hkx, const int servoSerie, const int servoModel)
{
    ServoHerkuleX *servo = static_cast <ServoHerkuleX *>(servo_hkx);

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

    int cwlimit = servo->getCwAngleLimit(REGISTER_ROM);
    int ccwlimit = servo->getCcwAngleLimit(REGISTER_ROM);
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),1)->setText(QString::number(cwlimit));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),2)->setText(QString::number(servo->getCwAngleLimit(REGISTER_RAM)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),1)->setText(QString::number(ccwlimit));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),2)->setText(QString::number(servo->getCcwAngleLimit(REGISTER_RAM)));
    ui->cwlimit->setValue(cwlimit);
    ui->ccwlimit->setValue(ccwlimit);
    ui->cwlimit_label->setText(tr("MIN position (") + QString::number(cwlimit) + ")");
    ui->ccwlimit_label->setText(tr("MAX position (") + QString::number(ccwlimit) + ")");
    if (cwlimit == 0 && ccwlimit == 0) // FIXME change these test conditions, they are for dynamixel
    {
        // wheel mode
        ui->cwlimit->setDisabled(true);
        ui->ccwlimit->setDisabled(true);
        ui->radioButton_2->setChecked(true);
    }
    else
    {
        // joint mode
        ui->cwlimit->setEnabled(true);
        ui->ccwlimit->setEnabled(true);
        ui->radioButton_1->setChecked(true);
    }

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
    int torque_enable = servo->getValue(REG_TORQUE_ENABLE);
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_ENABLE),2)->setText(QString::number(torque_enable));
    ui->torque_checkBox->setChecked(torque_enable);
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED),2)->setText(QString::number(servo->getValue(REG_LED)));

    double ctemp = servo->getCurrentTemperature();
    if (ctemp > servo->getHighestLimitTemp(REGISTER_ROM))
    {
        // Warn if servo temperature is too high
        ui->ctemp->setPalette(QPalette(QColor(255,0,0)));
    }
    else
    {
        ui->ctemp->setPalette(QPalette(QColor(85,85,127)));
    }

    ui->ctemp->display(ctemp);
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_TEMPERATURE),2)->setText(QString::number(ctemp, 'g', 3) + " *");

    double cvolt = servo->getCurrentVoltage();
    ui->cvolt->display(cvolt);
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_VOLTAGE),2)->setText(QString::number(cvolt, 'g', 3) + " V");
    if (cvolt < servo->getLowestLimitVolt(REGISTER_ROM) || cvolt > servo->getHighestLimitVolt(REGISTER_ROM))
    {
        // Warn if servo voltage is out of boundaries
        ui->cvolt->setPalette(QPalette(QColor(255,0,0)));
    }
    else
    {
        ui->cvolt->setPalette(QPalette(QColor(85,85,127)));
    }

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_CONTROL_MODE),2)->setText(QString::number(servo->getValue(REG_CURRENT_CONTROL_MODE)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TICK),2)->setText(QString::number(servo->getValue(REG_TICK)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CALIBRATED_POSITION),2)->setText(QString::number(servo->getValue(REG_CALIBRATED_POSITION)));
    int cpos = servo->getValue(REG_ABSOLUTE_POSITION);
    ui->cpos->setValue(cpos);
    ui->cpos_label->setText(tr("Current (") + QString::number(cpos) + ")");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_POSITION),2)->setText(QString::number(cpos));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DIFFERENTIAL_POSITION),2)->setText(QString::number(servo->getValue(REG_DIFFERENTIAL_POSITION)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_PWM),2)->setText(QString::number(servo->getValue(REG_PWM)));
    if (servoModel == SERVO_DRS_0402 || servoModel == SERVO_DRS_0602)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_2nd_POSITION),2)->setText(QString::number(servo->getValue(REG_ABSOLUTE_2nd_POSITION)));
    }
    int gpos = servo->getValue(REG_ABSOLUTE_GOAL_POSITION);
    ui->gpos->setValue(gpos);
    ui->gpos_label->setText(tr("Goal (") + QString::number(gpos) + ")");
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ABSOLUTE_GOAL_POSITION),2)->setText(QString::number(gpos));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_TRAJECTORY),2)->setText(QString::number(servo->getValue(REG_GOAL_TRAJECTORY)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_VELOCITY),2)->setText(QString::number(servo->getValue(REG_GOAL_VELOCITY)));
}

void MainWindow::updateRegisterTableDynamixel(Servo *servo_dxl, const int servoSerie, const int servoModel)
{
    ServoDynamixel *servo = static_cast <ServoDynamixel *>(servo_dxl);

    // EEPROM
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MODEL_NUMBER),1)->setText(QString::fromStdString(servo->getModelString()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_FIRMWARE_VERSION),1)->setText(QString::number(servo->getFirmwareVersion()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_ID),1)->setText(QString::number(servo->getValue(REG_ID)));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_BAUD_RATE),1)->setText(QString::number(servo->getBaudRate()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_RETURN_DELAY_TIME),1)->setText(QString::number(servo->getReturnDelay()));

    int cwlimit = servo->getCwAngleLimit();
    int ccwlimit = servo->getCcwAngleLimit();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MIN_POSITION),1)->setText(QString::number(cwlimit));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_POSITION),1)->setText(QString::number(ccwlimit));
    ui->cwlimit->setValue(cwlimit);
    ui->ccwlimit->setValue(ccwlimit);
    ui->cwlimit_label->setText(tr("MIN position (") + QString::number(cwlimit) + ")");
    ui->ccwlimit_label->setText(tr("MAX position (") + QString::number(ccwlimit) + ")");
    if (cwlimit == 0 && ccwlimit == 0)
    {
        // wheel mode
        ui->cwlimit->setDisabled(true);
        ui->ccwlimit->setDisabled(true);
        ui->radioButton_2->setChecked(true);
    }
    else
    {
        // joint mode
        ui->cwlimit->setEnabled(true);
        ui->ccwlimit->setEnabled(true);
        ui->radioButton_1->setChecked(true);
    }

    if (servoSerie == SERVO_EX || servoModel == SERVO_MX106)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_DRIVE_MODE),1)->setText(QString::number(static_cast<ServoMX*>(servo)->getDriveMode()));
    }
    else if (servoSerie == SERVO_XL || servoModel == SERVO_XL320)
    {
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CONTROL_MODE),1)->setText(QString::number(static_cast<ServoXL*>(servo)->getControlMode()));
    }
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TEMPERATURE_LIMIT),1)->setText(QString::number(servo->getHighestLimitTemp()) + " *");
    double dlow_volt_limit = servo->getLowestLimitVolt();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_LOWEST_LIMIT),1)->setText(QString::number(dlow_volt_limit) + " V");
    double dhigh_volt_limit = servo->getHighestLimitVolt();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_VOLTAGE_HIGHEST_LIMIT),1)->setText(QString::number(dhigh_volt_limit) + " V");
    int max_torque = servo->getMaxTorque();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_MAX_TORQUE),1)->setText(QString::number(max_torque));
    ui->maxTorqueSlider->setValue(max_torque);
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
    int torque_enable = servo->getTorqueEnabled();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_ENABLE),1)->setText(QString::number(torque_enable));
    ui->torque_checkBox->setChecked(torque_enable);
    int led = servo->getLed();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_LED),1)->setText(QString::number(led));
    ui->led_checkBox->setChecked(led);
    if (servoSerie != SERVO_MX && servoSerie != SERVO_XL)
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
    int gpos = servo->getGoalPosition();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_POSITION),1)->setText(QString::number(gpos));
    ui->gpos->setValue(gpos);
    ui->gpos_label->setText(tr("Goal (") + QString::number(gpos) + ")");
    int mspeed = servo->getMovingSpeed();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_GOAL_SPEED),1)->setText(QString::number(mspeed));
    ui->movingSpeedSlider->setValue(mspeed);
    if (servoModel != SERVO_XL320)
    {
        int tlimit = servo->getTorqueLimit();
        ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_TORQUE_LIMIT),1)->setText(QString::number(tlimit));
        ui->torqueLimitSlider->setValue(tlimit);
    }
    int cpos = servo->getCurrentPosition();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_POSITION),1)->setText(QString::number(cpos));
    ui->cpos->setValue(cpos);
    ui->cpos_label->setText(tr("Current (") + QString::number(cpos) + ")");

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_SPEED),1)->setText(QString::number(servo->getCurrentSpeed()));
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_LOAD),1)->setText(QString::number(servo->getCurrentLoad()));
    double dcvolt = servo->getCurrentVoltage();
    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_VOLTAGE),1)->setText(QString::number(dcvolt) + " V");
    ui->cvolt->display(dcvolt);
    if (dcvolt < dlow_volt_limit || dcvolt > dhigh_volt_limit)
    {
        // Warn if servo voltage is out of boundaries
        ui->cvolt->setPalette(QPalette(QColor(255,0,0)));
    }
    else
    {
        ui->cvolt->setPalette(QPalette(QColor(85,85,127)));
    }

    double ctemp = servo->getCurrentTemperature();
    if (ctemp > servo->getHighestLimitTemp())
    {
        // Warn if servo temperature is too high
        ui->ctemp->setPalette(QPalette(QColor(255,0,0)));
    }
    else
    {
        ui->ctemp->setPalette(QPalette(QColor(85,85,127)));
    }

    ui->tableWidget->item(getTableIndex(servoSerie, servoModel, REG_CURRENT_TEMPERATURE),1)->setText(QString::number(ctemp) + " *");
    ui->ctemp->display(ctemp);

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
}

void MainWindow::errorHandling(Servo *servo, const int servoSerie, const int servoModel)
{
    QString css_comm_ok("color: white;\nborder: 1px solid rgb(85, 170, 0);\nbackground: rgba(85, 200, 0, 128);");
    QString css_error("border: 1px solid rgb(255, 53, 3);\nbackground: rgba(255, 170, 0, 128);\ncolor: white;");
    QString css_status("border-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(255, 248, 191, 128);\ncolor: rgb(246, 130, 9);");
    QString css_ok_middle("border-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");
    QString css_ok_left("border-left: 1px solid rgb(10, 100, 255);\nborder-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");
    QString css_ok_right("border-right: 1px solid rgb(10, 100, 255);\nborder-top: 1px solid rgb(10, 100, 255);\nborder-bottom: 1px solid rgb(10, 100, 255);\nbackground: rgba(12, 170, 255, 128);\ncolor: white;");

    ControllerAPI *ctrl = NULL;
    if (getCurrentController(ctrl) > 0)
    {
        if (ctrl->getErrorCount() > 0)
        {
            ui->serialLinkErrors_label2->setStyleSheet(css_error);
        }
        else
        {
            ui->serialLinkErrors_label2->setStyleSheet(css_comm_ok);
        }
    }

    if (servo)
    {
        int srv_error = servo->getError();
        if (servoSerie >= SERVO_HERKULEX)
        {
            int srv_status = servo->getStatus();

            if (srv_error & ERRBIT_VOLTAGE)
                { ui->label_err_vin_2->setStyleSheet(css_error); }
            else
                { ui->label_err_vin_2->setStyleSheet(css_ok_left); }

            if (srv_error & ERRBIT_ALLOWED_POT)
                { ui->label_err_angle_2->setStyleSheet(css_error); }
            else
                { ui->label_err_angle_2->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_OVERHEAT)
                { ui->label_err_heat_2->setStyleSheet(css_error); }
            else
                { ui->label_err_heat_2->setStyleSheet(css_ok_middle); }

            if (srv_status & STATBIT_MOVING)
                { ui->label_stat_moving->setStyleSheet(css_status); }
            else
                { ui->label_stat_moving->setStyleSheet(css_ok_left); }

            if (srv_status & STATBIT_INPOSITION)
                { ui->label_stat_inpos->setStyleSheet(css_status); }
            else
                { ui->label_stat_inpos->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_INVALID_PKT)
            {
                ui->label_err_packet->setStyleSheet(css_status);

                if (srv_status & STATBIT_CHECKSUM_FLAG)
                    { ui->label_stat_crc->setStyleSheet(css_error); }
                else
                    { ui->label_stat_crc->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_UNKWOWN_CMD)
                    { ui->label_stat_cmd->setStyleSheet(css_error); }
                else
                    { ui->label_stat_cmd->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_RANGE)
                    { ui->label_stat_addr->setStyleSheet(css_error); }
                else
                    { ui->label_stat_addr->setStyleSheet(css_ok_middle); }

                if (srv_status & STATBIT_GARBAGE)
                    { ui->label_stat_gbg->setStyleSheet(css_error); }
                else
                    { ui->label_stat_gbg->setStyleSheet(css_ok_middle); }
            }
            else
            { ui->label_err_packet->setStyleSheet(css_ok_middle); }

            if (srv_status & STATBIT_TORQUE_ON)
                { ui->label_stat_ton->setStyleSheet(css_status); }
            else
                { ui->label_stat_ton->setStyleSheet(css_ok_right); }

            if (srv_error & ERRBIT_OVERLOAD)
                { ui->label_err_overload_2->setStyleSheet(css_error); }
            else
                { ui->label_err_overload_2->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_DRIVER_FAULT)
                { ui->label_err_driver->setStyleSheet(css_error); }
            else
                { ui->label_err_driver->setStyleSheet(css_ok_middle); }

            if (srv_error & ERRBIT_EEP_REG_DIST)
                { ui->label_err_eep->setStyleSheet(css_error); }
            else
                { ui->label_err_eep->setStyleSheet(css_ok_right); }
        }
        else
        {
            if (servoSerie >= SERVO_XL)
            {
                if (srv_error == ERRBIT2_RESULT)
                    { ui->label_err_res->setStyleSheet(css_error); }
                else
                    { ui->label_err_res->setStyleSheet(css_ok_left); }

                if (srv_error == ERRBIT2_INSTRUCTION)
                    { ui->label_err_cmd_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_cmd_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_CHECKSUM)
                    { ui->label_err_crc_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_crc_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_RANGE)
                    { ui->label_err_range_2->setStyleSheet(css_error); }
                else
                    { ui->label_err_range_2->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_LENGTH)
                    { ui->label_err_length->setStyleSheet(css_error); }
                else
                    { ui->label_err_length->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_DATA_LIMIT)
                    { ui->label_err_limit->setStyleSheet(css_error); }
                else
                    { ui->label_err_limit->setStyleSheet(css_ok_middle); }

                if (srv_error == ERRBIT2_ACCESS)
                    { ui->label_err_access->setStyleSheet(css_error); }
                else
                    { ui->label_err_access->setStyleSheet(css_ok_right); }
            }
            else
            {
                if (srv_error & 0x01)
                    { ui->label_err_vin->setStyleSheet(css_error); }
                else
                    { ui->label_err_vin->setStyleSheet(css_ok_right); }

                if (srv_error & 0x02)
                    { ui->label_err_angle->setStyleSheet(css_error); }
                else
                    { ui->label_err_angle->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x04)
                    { ui->label_err_heat->setStyleSheet(css_error); }
                else
                    { ui->label_err_heat->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x08)
                    { ui->label_err_range->setStyleSheet(css_error); }
                else
                    { ui->label_err_range->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x10)
                    { ui->label_err_crc->setStyleSheet(css_error); }
                else
                    { ui->label_err_crc->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x20)
                    { ui->label_err_overload->setStyleSheet(css_error); }
                else
                    { ui->label_err_overload->setStyleSheet(css_ok_middle); }

                if (srv_error & 0x40)
                    { ui->label_err_cmd->setStyleSheet(css_error); }
                else
                    { ui->label_err_cmd->setStyleSheet(css_ok_left); }
            }
        }
    }
}

int MainWindow::getRegisterNameFromTableIndex(const int servo_serie, const int servo_model, int table_index)
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

int MainWindow::getTableIndex(const int servo_serie, const int servo_model, int reg_name)
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

void MainWindow::generateRegisterTable(const int servo_serie, const int servo_model)
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
        if (servo_serie != tableServoSerie)
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
        if (servo_serie != tableServoSerie)
        {
            ui->tableWidget->setColumnHidden(2, true);
            ui->tableWidget->horizontalHeaderItem(1)->setText(tr("Value"));
        }

        // Generate new table
        generateRegisterTableDynamixel(servo_serie, servo_model);
    }

    // Save table params
    tableServoSerie = servo_serie;
    tableServoModel = servo_model;

    // Update
    ui->tableWidget->update();
}

void MainWindow::generateRegisterTableHerkuleX(const int servo_serie, const int servo_model)
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

void MainWindow::generateRegisterTableDynamixel(const int servo_serie, const int servo_model)
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

void MainWindow::cleanRegisterTable()
{
    QObject::disconnect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(modifier(int, int)));

    for (int i = 0; i < ui->tableWidget->rowCount(); i++)
    {
        ui->tableWidget->item(i, 1)->setText("");
        if (tableServoSerie >= SERVO_HERKULEX)
        {
            ui->tableWidget->item(i, 2)->setText("");
        }
    }

    QObject::connect(ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(modifier(int, int)));
}

void MainWindow::toggleServoPanel(bool status)
{
    if (status == false)
    {
        // Controls
        ui->radioButton_1->setChecked(false);
        ui->radioButton_2->setChecked(false);
        ui->cvolt->display(0);
        ui->cvolt->setPalette(QPalette(QColor(85,85,127)));
        ui->ctemp->display(0);
        ui->ctemp->setPalette(QPalette(QColor(85,85,127)));
        ui->torque_checkBox->setChecked(0);
        ui->led_checkBox->setChecked(0);
        ui->maxTorqueSlider->setValue(0);
        ui->torqueLimitSlider->setValue(0);
        ui->movingSpeedSlider->setValue(0);
        ui->cwlimit->setValue(0);
        ui->cwlimit_label->setText(tr("MIN position"));
        ui->ccwlimit->setValue(0);
        ui->ccwlimit_label->setText(tr("MAX position"));
        ui->cpos->setValue(0);
        ui->cpos_label->setText(tr("Current position"));
        ui->gpos->setValue(0);
        ui->gpos_label->setText(tr("Goal position"));

        // Infos
        QPixmap servoDefaultIcon(":/icons/icons/emblem-unreadable.svg");
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
        servoDefaultIcon.setDevicePixelRatio(qApp->devicePixelRatio());
#endif
        ui->servoPicture_label->setPixmap(servoDefaultIcon);
        ui->textBrowser_spec->setText(tr("No documentation available..."));
        ui->servoManual_label->hide();
        ui->copyrightNotice_label->hide();
    }

    // Controls
    ui->radioButton_1->setEnabled(status);
    ui->radioButton_2->setEnabled(status);
    ui->torque_checkBox->setEnabled(status);
    ui->led_checkBox->setEnabled(status);
    ui->maxTorqueSlider->setEnabled(status);
    ui->torqueLimitSlider->setEnabled(status);
    ui->movingSpeedSlider->setEnabled(status);
    ui->cwlimit->setEnabled(status);
    ui->ccwlimit->setEnabled(status);
    ui->gpos->setEnabled(status);

    // Status box
    ui->frameStatus->setVisible(status);

    // Infos
    ui->servoManual_label->setVisible(status);
    ui->copyrightNotice_label->setVisible(status);
}

/* ************************************************************************** */

void MainWindow::advanceScannerStart()
{
    // Check if a controller is not currently busy scanning or reading
    for (auto p: serialPorts)
    {
        if (p->deviceController != NULL)
        {
            if (!(p->deviceController->getState() == state_stopped ||
                  p->deviceController->getState() == state_scanned ||
                  p->deviceController->getState() == state_ready))
            {
                return;
            }
        }
    }

    if (asw == NULL)
    {
        // Pause controllers
        for (auto p: serialPorts)
        {
            if (p->deviceController != NULL)
            {
                p->deviceController->pauseThread();
            }
        }

        // Lock current window
        this->setDisabled(true);

        // Start an advance scanner window, lock current windows
        asw = new AdvanceScanner(this);
        asw->show();
        asw->resizeTables();
    }
}

void MainWindow::advanceScannerStop()
{
    if (asw != NULL)
    {
        // Un-pause controllers
        for (auto p: serialPorts)
        {
            if (p->deviceController != NULL)
            {
                p->deviceController->pauseThread();
            }
        }

        // Un-lock current window
        this->setDisabled(false);

        // Delete the advance scanner window
        delete asw;
        asw = NULL;
    }
}

/* ************************************************************************** */

void MainWindow::settingsStart()
{
    if (stw == NULL)
    {
        stw = new Settings();
        stw->show();
    }
    else
    {
        stw->show();
    }
}

void MainWindow::settingsStop()
{
    if (asw != NULL)
    {
        asw->hide();
    }
}

/* ************************************************************************** */

void MainWindow::about()
{
    QAbout *qa = new QAbout();
    qa->exec();
}

void MainWindow::aboutQt()
{
    QMessageBox::aboutQt(this, tr("About Qt"));
}

/* ************************************************************************** */

void MainWindow::resetServo()
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        // Build message box
        QMessageBox confirmationBox;
        confirmationBox.setIcon(QMessageBox::Warning);
        confirmationBox.setWindowTitle(tr("Reset servo to factory settings?"));
        confirmationBox.setText(tr("Are you sure you want to reset servo #") + QString::number(s->getId()) + tr(" to factory settings?\nAll settings will be reseted to default value!"));

        confirmationBox.addButton(QMessageBox::No);
        confirmationBox.addButton(tr("Yes, except ID and baudrate"), QMessageBox::YesRole);
        confirmationBox.addButton(tr("Yes, except ID"), QMessageBox::YesRole);
        confirmationBox.addButton(QMessageBox::Yes);

        // Execute message box
        int ret = confirmationBox.exec();
        if (ret == QMessageBox::No)
        {
            return;
        }
        else
        {
            if (ret == QMessageBox::Yes)
            {
                s->reset(RESET_ALL);
            }
            else if (ret == 1)
            {
                s->reset(RESET_ALL_EXCEPT_ID);
            }
            else //if (ret == 0)
            {
                s->reset(RESET_ALL_EXCEPT_ID_BAUDRATE);
            }
        }
    }
}

void MainWindow::rebootServo()
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        s->reboot();
    }
}

void MainWindow::refreshServo()
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        s->refresh();
    }
}

/* ************************************************************************** */

void MainWindow::clearErrors()
{
    ControllerAPI *c = NULL;

    if (getCurrentController(c) > 0)
    {
        c->clearErrorCount();
    }

    Servo *s = NULL;

    if (getCurrentServo(s) > 0)
    {
        s->clearErrors();
    }
}

/* ************************************************************************** */

void MainWindow::toggleRunningMode()
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        if (ui->radioButton_1->isChecked() == true)
        {
            // Check if we are already in joint mode with default angle limits
            if (!((s->getCwAngleLimit() == 0) && (s->getCcwAngleLimit() == (s->getSteps() - 1))))
            {
                //std::cout << "toggleTorque(#" << s->getId() << " / let's go to JOINT MODE)" << std::endl;

                // Ask for confirmation
                if ((s->getCwAngleLimit() != 0) ||
                    (s->getCcwAngleLimit() != 0 && s->getCcwAngleLimit() != (s->getSteps() - 1)))
                {
                    int ret = QMessageBox::warning(this, tr("Switch to 'joint mode'"),
                                                   tr("You currently have custom angle limits. Switching to 'joint mode' will reset them to 0 and ")
                                                   + QString::number(s->getSteps() - 1) + tr(".\nAre you sure you want to continue?"),
                                                   QMessageBox::Yes | QMessageBox::No);
                    if (ret == QMessageBox::No)
                    {
                        return;
                    }
                }

                s->setCWLimit(0);
                s->setCCWLimit(s->getSteps() - 1);
                ui->cwlimit_label->setText(tr("MIN position (") + QString::number(0) + ")");
                ui->ccwlimit_label->setText(tr("MAX position (") + QString::number(s->getSteps() - 1) + ")");
            }
        }
        else
        {
            // Check if we are already in wheel mode with default angle limits
            if (!(s->getCwAngleLimit() == 0 && s->getCcwAngleLimit() == 0))
            {
                //std::cout << "toggleTorque(#" << s->getId() << " / let's go to WHEEL MODE)" << std::endl;

                // Ask for confirmation
                if (s->getCwAngleLimit() != 0 || s->getCcwAngleLimit() != (s->getSteps() - 1))
                {
                    int ret = QMessageBox::warning(this, tr("Switch to 'wheel mode'"),
                                                   tr("You currently have custom angle limits. Switching to 'wheel mode' will reset them both to 0.\n"
                                                      "Are you sure you want to continue?"),
                                                   QMessageBox::Yes | QMessageBox::No);
                    if (ret == QMessageBox::No)
                    {
                        return;
                    }
                }

                //s->setMovingSpeed(0);
                s->setCWLimit(0);
                s->setCCWLimit(0);
                ui->cwlimit_label->setText(tr("MIN position (") + QString::number(0) + ")");
                ui->ccwlimit_label->setText(tr("MAX position (") + QString::number(0) + ")");
            }
        }
    }
}

void MainWindow::toggleTorque(bool torque)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "toggleTorque(#" << s->getId() << " / torque:" << torque << ")" << std::endl;

        // Enable Torque
        s->setTorqueEnabled(torque);
    }
}

void MainWindow::toggleLED(bool led)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "toggleLED(#" << s->getId() << " / led:" << led << ")" << std::endl;

        // Enable LED
        s->setLed(led);
    }
}

void MainWindow::moveMaxTorque(int torque)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveMaxTorque(#" << s->getId() << " / torque:" << torque << ")" << std::endl;

        // Update max torque
        if (tableServoSerie < SERVO_HERKULEX)
        {
            static_cast<ServoDynamixel*>(s)->setMaxTorque(torque);
        }
    }
}

void MainWindow::moveTorqueLimit(int limit)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveTorqueLimit(#" << s->getId() << " / limit:" << limit << ")" << std::endl;

        // Update torque limit
        if (tableServoSerie == SERVO_XL)
        {
            s->setValue(REG_GOAL_TORQUE, limit);
        }
        else
        {
            s->setValue(REG_TORQUE_LIMIT, limit);
        }
    }
}

void MainWindow::moveMovingSpeed(int speed)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveMovingSpeed(#" << s->getId() << " / moving speed:" << speed << ")" << std::endl;

        // Update moving speed
        if (tableServoSerie < SERVO_HERKULEX)
        {
            static_cast<ServoDynamixel*>(s)->setMovingSpeed(speed);
        }
    }
}

void MainWindow::moveCWLimit(int limit)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveCWLimit(#" << s->getId() << " / limit:" << limit << ")" << std::endl;

        s->setValue(REG_MIN_POSITION, limit, REGISTER_BOTH);

        // Update label
        ui->cwlimit_label->setText(tr("MIN position (") + QString::number(limit) + ")");
    }
}

void MainWindow::moveCCWLimit(int limit)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveCCWLimit(#" << s->getId() << " / limit:" << limit << ")" << std::endl;

        s->setValue(REG_MAX_POSITION, limit, REGISTER_BOTH);

        // Update label
        ui->ccwlimit_label->setText(tr("MAX position (") + QString::number(limit) + ")");
    }
}

void MainWindow::moveServo(int goal)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveServo(#" << s->getId() << " / goal:" << goal << ")" << std::endl;

        s->setGoalPosition(goal);

        // Update label
        ui->gpos_label->setText(tr("Goal (") + QString::number(goal) + ")");
    }
}

void MainWindow::modifier(int row, int column)
{
    Servo *s = NULL;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        int reg_value = ui->tableWidget->item(row, column)->text().toInt();
        int reg_name = getRegisterNameFromTableIndex(tableServoSerie, tableServoModel, row);
        int reg_type = REGISTER_AUTO;

        if (tableServoSerie >= SERVO_HERKULEX)
        {
            if (column == 1)
                reg_type = REGISTER_ROM;
            else
                reg_type = REGISTER_RAM;
        }

        //std::cout << "MODIFIER (" << row << " x " << column << ") name: " << getRegisterNameTxt(reg_name) << "  value: " << reg_value << "  (ROM/RAM: " << reg_type << ")" << std::endl;

        s->setValue(reg_name, reg_value, reg_type);

        // Changing the ID of a device mean special treatment...
        if (reg_name == REG_ID)
        {
            // Proceed in every case except if we are trying to change a ROM ID for an HerkuleX device
            if (s->getDeviceBrand() == SERVO_HERKULEX && reg_type == REGISTER_ROM)
            {
                return;
            }

            // Get selected item, and change its text
            // It's important because we rely on this text to parse ID from the deviceTreeWidget
            if (ui->deviceTreeWidget->selectedItems().size() > 0)
            {
                QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

                // Check if the item exist, plus if this a device and not a port
                if (item != NULL && item->parent() != NULL)
                {
                    // Finaly change the item text
                    QString device_txt = "[#" + QString::number(reg_value) + "]  " + QString::fromStdString(s->getModelString());
                    item->setText(0, device_txt);
                }
            }
        }
    }
}
