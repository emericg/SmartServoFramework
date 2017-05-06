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
#include "widgetSerialScan.h"
#include "qabout.h"

#include "advancescanner.h"
#include "settings.h"

// SmartServoFramework
#include "../../src/DynamixelController.h"
#include "../../src/HerkuleXController.h"
#include "../../src/DynamixelTools.h"
#include "../../src/HerkuleXTools.h"

// Qt
#include <QApplication>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QTextStream>
#include <QTextBrowser>
#include <QMessageBox>
#include <QSignalMapper>
#include <QCloseEvent>
#include <QFile>
#include <QTimer>

// C++ standard libraries
#include <iostream>

/* ************************************************************************** */

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // UI
    ui->setupUi(this);
    ui->deviceTreeWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    ui->serialPortErrors_label->hide();
    ui->widgetStatus->hide();

    // Force custom window size
    setGeometry(0, 0, 1200, 640);

    // Save "serial" and "loading" tab widgets
    serialTabWidget = ui->tabWidget->widget(ui->tabWidget->count() - 2); // 5th tab
    loadingTabWidget = ui->tabWidget->widget(ui->tabWidget->count() - 1); // 6th (and last) tab

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

    QObject::connect(ui->widgetStatus, SIGNAL(updateButton()), this, SLOT(clearErrors()));
    QObject::connect(ui->widgetStatus, SIGNAL(clearButton()), this, SLOT(clearErrors()));

    QObject::connect(ui->pushScanSerial, SIGNAL(clicked()), this, SLOT(scanSerialPorts()));
    QObject::connect(ui->pushScanServo, SIGNAL(clicked()), this, SLOT(scanServos()));
    QObject::connect(ui->deviceTreeWidget, SIGNAL(itemSelectionChanged()), this, SLOT(treewidgetSelection()));

    // Resize content when changing tab
    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(resizeTabWidgetContent()));

    // Initialize device "self refresh" loop
    selfRefreshTimer = new QTimer(this);
    connect(selfRefreshTimer, SIGNAL(timeout()), this, SLOT(servoUpdate()));

    // Setting window
    stw = new Settings();
    stw->readSettings();
    stw->loadSettings();

#ifdef Q_OS_OSX
    ui->toolBar->setStyleSheet("");
#endif
}

MainWindow::~MainWindow()
{
    if (asw)
    {
        asw->close();
        delete asw;
        asw = nullptr;
    }

    if (stw)
    {
        stw->close();
        delete stw;
        stw = nullptr;
    }

    delete ui;
    delete selfRefreshTimer;

    // Clean up controllers
    for (auto p: serialPorts)
    {
        if (p != nullptr)
        {
            if (p->deviceController != nullptr)
            {
                delete p->deviceController;
                p->deviceController = nullptr;
            }

            delete p;
            p = nullptr;
        }
    }
}

/* ************************************************************************** */

void MainWindow::helpScreen(bool loading)
{
    ui->tabWidget->findChild<QTabBar *>()->setVisible(false);
    ui->tabWidget->setDocumentMode(true);

    bool tabalreadythere = false;
    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "serial")
            ui->tabWidget->removeTab(i);
    }
    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "loading")
            tabalreadythere = true;
    }
    if (tabalreadythere == false)
        ui->tabWidget->addTab(loadingTabWidget, "loading");

    // Set tab index
    ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);

    // Loading bar
     ui->frame_loading->setVisible(loading);

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

void MainWindow::serialScreen()
{
    ui->tabWidget->findChild<QTabBar *>()->hide();
    ui->tabWidget->setDocumentMode(true);

    bool tabalreadythere = false;
    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "loading")
            ui->tabWidget->removeTab(i);
    }
    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "serial")
            tabalreadythere = true;
    }
    if (tabalreadythere == false)
    {
        ui->tabWidget->addTab(serialTabWidget, "serial");
    }

    // Set tab index
    ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);
}

void MainWindow::servoScreen()
{
    // Set the QTabBar visible
    ui->tabWidget->findChild<QTabBar *>()->setVisible(true);
    ui->tabWidget->setDocumentMode(false);

    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "serial")
            ui->tabWidget->removeTab(i);
    }
    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        if (ui->tabWidget->tabText(i) == "loading")
            ui->tabWidget->removeTab(i);
    }

    // Set tab index
    ui->tabWidget->setCurrentIndex(0);
}

/* ************************************************************************** */

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
    ui->frameDevices->setDisabled(true);
    helpScreen(true);

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
        if (p != nullptr)
        {
            if (p->deviceController != nullptr)
            {
                delete p->deviceController;
                p->deviceController = nullptr;
            }

            delete p;
            p = nullptr;
        }
    }
    serialPorts.clear();

    // Scan for available/new serial ports
    std::vector <std::string> availablePorts;

#if defined(FEATURE_QTSERIAL)
    serialPortsScannerQt(availablePorts);
#else
    serialPortsScanner(availablePorts);
#endif

    if (availablePorts.size() == 0)
    {
        ui->serialPortErrors_label->show();

        QTreeWidgetItem *port = new QTreeWidgetItem();
        port->setText(0, tr("No serial port available!"));
        ui->deviceTreeWidget->addTopLevelItem(port);
    }
    else
    {
        ui->serialPortErrors_label->hide();

        // Create "helper" structure to keep track of serial port instances
        // Create an entry inside the "scan group box" for each serial port
        for (auto p: availablePorts)
        {
            SerialPortHelper *helper = new SerialPortHelper();
            serialPorts.push_back(helper);

            helper->deviceName_str = p;
            helper->deviceName_qstr = QString::fromStdString(p);
            helper->deviceWidget = new widgetSerialScan(helper->deviceName_qstr);

            QObject::connect(helper->deviceWidget, SIGNAL(scanButton(QString)), this, SLOT(refreshSerialPort(QString)));

            ui->groupPorts->layout()->addWidget(helper->deviceWidget);
            ui->groupPorts->update();
        }

        // Update the GUI before each servo scan
        qApp->processEvents();

        // Launch servo scanning?
        if (autoscan == true)
            scanServos();
        else
            helpScreen(false);
    }

    // Indicate that we are no longer scanning
    ui->frameDevices->setEnabled(true);

    // Maybe this scan did not yield any results, to avoid showing a blank
    // interface, we do not turn off loading screen, only the loading animation
    ui->frame_loading->hide();

    // Update UI
    ui->deviceTreeWidget->update();
}

void MainWindow::scanServos()
{
    ui->frameDevices->setDisabled(true);
    scan_running = true;
    treewidgetAutoSelection = false;

    for (SerialPortHelper *h: serialPorts)
    {
        if (scan_running == true)
        {
            // Process the device only if its GUI element is 'checked'
            if (h->deviceWidget->isSelected() == true)
            {
                // Launch a scan
                scanServos(h->deviceName_qstr);
            }
        }
    }

    scan_running = false;
    ui->frameDevices->setEnabled(true);
}

void MainWindow::refreshSerialPort(QString port_qstring)
{
    ui->frameDevices->setDisabled(true);
    scan_running = true;
    treewidgetAutoSelection = false;

    // Launch a scan
    scanServos(port_qstring);

    scan_running = false;
    ui->frameDevices->setEnabled(true);
}

void MainWindow::scanServos(QString port_qstring)
{
    // Disable scan buttons
    ui->frameDevices->setDisabled(true);

    bool ctrl_locks = stw->getLock();
    int ctrl_freq = stw->getFreq();

    // First locate the port to scan
    for (auto h: serialPorts)
    {
        if (h->deviceName_qstr == port_qstring)
        {
            std::cout << ">> scanServos(" << h->deviceName_str << ")" << std::endl;

            // Indicate we are starting to scan
            {
                helpScreen(true);
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
            QTreeWidgetItem *port = nullptr;
            for (int i = 0; i < ui->deviceTreeWidget->topLevelItemCount(); i++)
            {
                if (ui->deviceTreeWidget->topLevelItem(i)->text(0) == h->deviceName_qstr)
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
            if (port == nullptr)
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
                if (h->deviceWidget->getCurrentIndex() == 0)
                {
                    // Rewrite 'scan_setting' if scan preset is set to 'auto'
                    int scanrounds_item[4] = {2, 4, 5, 8};
                    scan_rounds = 4;
                    scan_results = 0;
                    scan_setting = scanrounds_item[i];
                }
                else
                {
                    scan_rounds  = 1;
                    scan_results = 0;
                    scan_setting = h->deviceWidget->getCurrentIndex();

                    // We need to check is the protocol or speed has changed on the selected port
                    deviceControllerProtocolSAVED = h->deviceControllerProtocol;
                    deviceControllerSpeedSAVED = h->deviceControllerSpeed;
                }

                // Load controller settings
                h->deviceControllerProtocol = h->deviceWidget->getCurrentProtocol(scan_setting);
                h->deviceControllerSpeed = h->deviceWidget->getCurrentSpeed(scan_setting);
                h->deviceControllerDeviceClass = h->deviceWidget->getCurrentDeviceClass(scan_setting);

                // Do we need a new controller for this serial port?
                // - no controller instanciated
                // - controller with new protocol or speed
                if (h->deviceController == nullptr ||
                    deviceControllerProtocolSAVED != h->deviceControllerProtocol ||
                    deviceControllerSpeedSAVED != h->deviceControllerSpeed)
                {
                    // Delete old controller (if needed)
                    if (h->deviceController != nullptr)
                    {
                        h->deviceController->disconnect();
                        delete h->deviceController;
                        h->deviceController = nullptr;
                    }

                    // Create a new controller
                    if (h->deviceControllerProtocol == PROTOCOL_DXLv1 ||
                        h->deviceControllerProtocol == PROTOCOL_DXLv2)
                    {
                        h->deviceController = new DynamixelController(ctrl_freq, h->deviceControllerDeviceClass);
                    }
                    else if (h->deviceControllerProtocol == PROTOCOL_HKX)
                    {
                        h->deviceController = new HerkuleXController(ctrl_freq, h->deviceControllerDeviceClass);
                    }

                    // Connect the controller to its serial port
                    if (h->deviceController != nullptr)
                    {
                        scan_results = h->deviceController->connect(h->deviceName_str, h->deviceControllerSpeed);

                        if (scan_results != 1)
                        {
                            h->deviceController->disconnect();
                            delete h->deviceController;
                            h->deviceController = nullptr;
                        }
                    }
                }

                // Scan
                if (scan_running == true && h->deviceController != nullptr)
                {
/*
                    // Are we scanning the currently selected port? Then go to the loading screen
                    ControllerAPI *ctrl = nullptr;
                    getCurrentController(ctrl);
                    if (h->deviceController == ctrl)
                    {
                        helpScreen(true);
                        qApp->processEvents();
                    }
*/
                    // Scan for servo(s)
                    h->deviceController->autodetect(h->deviceWidget->getMinRange(),
                                                    h->deviceWidget->getMaxRange());

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
                            if (h->deviceControllerProtocol == PROTOCOL_HKX)
                            {
                                device->setIcon(0, QIcon(":/devices/devices/HKX.svg"));
                            }
                            else
                            {
                                if (h->deviceControllerProtocol == PROTOCOL_DXLv2)
                                    device->setIcon(0, QIcon(":/devices/devices/DXL.svg")); // DXLv2.svg
                                else if (h->deviceControllerProtocol == PROTOCOL_DXLv1)
                                    device->setIcon(0, QIcon(":/devices/devices/DXL.svg")); // DXLv1.svg
                            }
                            device->setText(0, device_txt);

                            if (treewidgetAutoSelection == false)
                            {
                                // Autoselect first servo
                                ui->deviceTreeWidget->clearSelection();
                                device->setSelected(true);
                                treewidgetAutoSelection = true;

                                servoScreen();
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
                h->deviceController = nullptr;
            }

            // Unlock windows size
            this->setMinimumSize(0, 0);
            this->setMaximumSize(4096, 4096);
            ui->centralWidget->setMinimumSize(0, 0);
            ui->centralWidget->setMaximumSize(4096, 4096);

            // Refresh UI
            ui->deviceTreeWidget->update();
        }
    }

    ui->frameDevices->setEnabled(true);
}

/* ************************************************************************** */

int MainWindow::getCurrentController(ControllerAPI *&ctrl)
{
    int retcode = 0;

    // Get selected item
    if (ui->deviceTreeWidget->selectedItems().size() > 0)
    {
        QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

        // Check if the item exist, plus if this a device and not a port
        if (item != nullptr && item->parent() != nullptr)
        {
            // Print status?
            std::string port = item->parent()->text(0).toStdString();
            //std::cout << "> Selected serial port: " << port << std::endl;

            // Search for the port
            for (auto p: serialPorts)
            {
                // Found it
                if (p->deviceName_str == port)
                {
                    ctrl = p->deviceController;

                    if (ctrl != nullptr)
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
        if (item != nullptr && item->parent() != nullptr)
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
                if (p != nullptr && p->deviceName_str == port)
                {
                    // We have the servo
                    if (sid >= 0 && sid < 254)
                    {
                        if (p->deviceController != nullptr)
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

int MainWindow::getCurrentSerialPort(SerialPortHelper **port)
{
    int retcode = 0;

    // Get selected item
    if (ui->deviceTreeWidget->selectedItems().size() > 0)
    {
        QTreeWidgetItem *item = ui->deviceTreeWidget->selectedItems().at(0);

        // Check if the item exist, plus if this a device and not a port
        if (item != nullptr && item->parent() == nullptr)
        {
            std::string port_name = item->text(0).toStdString();

            // Search for the port into our serialPorts vector
            for (auto p: serialPorts)
            {
                // We found the port
                if (p != nullptr && p->deviceName_str == port_name)
                {
                    //std::cout << "> Serial port: " << port_name << std::endl;
                    *port = p;
                    retcode = 1;
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
        if (item != nullptr && item->parent() != nullptr)
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
                if (p != nullptr && p->deviceName_str == port)
                {
                    // We found the servo
                    if (sid >= 0 && sid < 254)
                    {
                        if (p->deviceController != nullptr)
                        {
                            servo = p->deviceController->getServo(sid);
                            if (servo != nullptr)
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

/* ************************************************************************** */

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
                if (p->deviceController != nullptr)
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
        if (p->deviceController != nullptr)
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
        ui->widgetRT->resizeTableWidget();
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

/* ************************************************************************** */

void MainWindow::treewidgetSelection()
{
    Servo *servo = nullptr;
    SerialPortHelper *serial = nullptr;

    // Get currently selected servo
    if (getCurrentServo(servo) > 0)
    {
        servoScreen();
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

        QObject::disconnect(ui->widgetRT, SIGNAL(cellChangedSignal(int,int)), this, SLOT(modifier(int, int)));

        // Adapt to servo model
        ////////////////////////////////////////////////////////////////////////

        int servoSerie, servoModel;
        servo->getModelInfos(servoSerie, servoModel);

        // Clean and generate a new register table (if we switch servo serie)
        // and device error box
        if (servoSerie != currentServoSerie && servoModel != currentServoModel)
        {
            currentServoSerie = servoSerie;
            currentServoModel = servoModel;
            ui->widgetRT->generateRegisterTable(servoSerie, servoModel);
            ui->widgetStatus->updateVisibility(servoSerie);

            if (servoSerie >= SERVO_HERKULEX)
            {
                ui->servoManual_label->setText("<html><head/><body><p><a href=\"http://hovis.co.kr/guide/herkulex_eng.html\"><span style=\" text-decoration: underline; color:#ffffff;\">Download manuals on Dongbu Robot website</span></a></p></body></html>");
                ui->copyrightNotice_label->setText("<html><head/><body><p>Technical specifications provided by <a href=\"www.dongburobot.com\"><span style=\" text-decoration: underline; color:#0000ff;\">dongburobot.com</span></a> website.</p></body></html>");
            }
            else
            {
                ui->servoManual_label->setText("<html><head/><body><p><a href=\"http://support.robotis.com/en/\"><span style=\" text-decoration: underline; color:#ffffff;\">Consult the online manuals on Robotis website</span></a></p></body></html>");
                ui->copyrightNotice_label->setText("<html><head/><body><p>Pictures and technical specifications courtesy of <a href=\"www.robotis.com\"><span style=\" text-decoration: underline; color:#0000ff;\">robotis.com</span></a></p></body></html>");
            }
        }

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

        // Model specific picture and doc
        QFile servoSpec;
        QPixmap servoIcon;
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

        case SERVO_X:
            servoIcon.load(":/devices/devices/X-Series.jpg");
            if (servoModel == SERVO_XH430_V210 || servoModel == SERVO_XH430_W210)
            {
                servoSpec.setFileName(":/specs/specs/XH-430-VW210.html");
            }
            if (servoModel == SERVO_XH430_V350 || servoModel == SERVO_XH430_W350)
            {
                servoSpec.setFileName(":/specs/specs/XH-430-VW350.html");
            }
            if (servoModel == SERVO_XM430_W210)
            {
                servoSpec.setFileName(":/specs/specs/XM-430-W210.html");
            }
            if (servoModel == SERVO_XM430_W350)
            {
                servoSpec.setFileName(":/specs/specs/XM-430-W350.html");
            }
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
        int pictureSize = ui->tabWidget->size().height() * 0.33; // All of our pictures are square
        ui->servoPicture_label->setMaximumWidth(pictureSize);
        ui->servoPicture_label->setMaximumHeight(pictureSize);
        ui->servoPicture_label->setScaledContents(true);
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
    else if (getCurrentSerialPort(&serial) > 0)
    {
        // Clean contextual menu
        ui->deviceTreeWidget->removeAction(refreshAction);
        ui->deviceTreeWidget->removeAction(rebootAction);
        ui->deviceTreeWidget->removeAction(resetAction);

        // Show serial screen
        serialScreen();

        // Populate the serial screen with relevent infos
        if (serial->deviceControllerDeviceClass > 0)
        {
            int deviceCount = 0;
            if (serial->deviceController)
                deviceCount = serial->deviceController->getServos().size();

            ui->widget_serial->setInfos(serial->deviceName_str,
                                        serial->deviceControllerSpeed,
                                        serial->deviceControllerProtocol,
                                        deviceCount);
        }

        resizeTabWidgetContent();
    }
    else
    {
        // Clean contextual menu
        ui->deviceTreeWidget->removeAction(refreshAction);
        ui->deviceTreeWidget->removeAction(rebootAction);
        ui->deviceTreeWidget->removeAction(resetAction);

        // Show help screen
        helpScreen(false);
    }
}

void MainWindow::serialportSelection()
{
    //
}

void MainWindow::servoSelection()
{
    //
}

void MainWindow::servoUpdate()
{
    Servo *servo = nullptr;

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

        QObject::disconnect(ui->widgetRT, SIGNAL(cellChangedSignal(int,int)), this, SLOT(modifier(int, int)));

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

        ControllerAPI *ctrl = nullptr;
        if (getCurrentController(ctrl) > 0)
        {
            ui->widgetStatus->handleErrors(ctrl, servo, servoSerie, servoModel);
        }

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

        QObject::connect(ui->widgetRT, SIGNAL(cellChangedSignal(int,int)), this, SLOT(modifier(int, int)));
    }
    else
    {
        // the "help" screen should be selected, so nothing to do...
    }

    // Update
    ui->widgetRT->updateTable();
}

/* ************************************************************************** */

void MainWindow::updateRegisterTableHerkuleX(Servo *servo_hkx, const int servoSerie, const int servoModel)
{
    ServoHerkuleX *servo = static_cast <ServoHerkuleX *>(servo_hkx);

    // Update register table
    ui->widgetRT->updateRegisterTableHerkuleX(servo_hkx, servoSerie, servoModel);

    // Update servo panel
    int cwlimit = servo->getCwAngleLimit(REGISTER_ROM);
    int ccwlimit = servo->getCcwAngleLimit(REGISTER_ROM);

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


    int torque_enable = servo->getValue(REG_TORQUE_ENABLE);
    ui->torque_checkBox->setChecked(torque_enable);

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

    double cvolt = servo->getCurrentVoltage();
    ui->cvolt->display(cvolt);
    if (cvolt < servo->getLowestLimitVolt(REGISTER_ROM) || cvolt > servo->getHighestLimitVolt(REGISTER_ROM))
    {
        // Warn if servo voltage is out of boundaries
        ui->cvolt->setPalette(QPalette(QColor(255,0,0)));
    }
    else
    {
        ui->cvolt->setPalette(QPalette(QColor(85,85,127)));
    }

    int cpos = servo->getValue(REG_ABSOLUTE_POSITION);
    ui->cpos->setValue(cpos);
    ui->cpos_label->setText(tr("Current (") + QString::number(cpos) + ")");

    int gpos = servo->getValue(REG_ABSOLUTE_GOAL_POSITION);
    ui->gpos->setValue(gpos);
    ui->gpos_label->setText(tr("Goal (") + QString::number(gpos) + ")");
}

void MainWindow::updateRegisterTableDynamixel(Servo *servo_dxl, const int servoSerie, const int servoModel)
{
    ServoDynamixel *servo = static_cast <ServoDynamixel *>(servo_dxl);

    // Update register table
    ui->widgetRT->updateRegisterTableDynamixel(servo_dxl, servoSerie, servoModel);

    // Update servo panel
    // EEPROM
    int cwlimit = servo->getCwAngleLimit();
    int ccwlimit = servo->getCcwAngleLimit();
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

    double dlow_volt_limit = servo->getLowestLimitVolt();
    double dhigh_volt_limit = servo->getHighestLimitVolt();
    int max_torque = servo->getMaxTorque();
    ui->maxTorqueSlider->setValue(max_torque);

    // RAM
    int torque_enable = servo->getTorqueEnabled();
    ui->torque_checkBox->setChecked(torque_enable);
    int led = servo->getLed();
    ui->led_checkBox->setChecked(led);

    int gpos = servo->getGoalPosition();
    ui->gpos->setValue(gpos);
    ui->gpos_label->setText(tr("Goal (") + QString::number(gpos) + ")");
    int mspeed = servo->getMovingSpeed();
    ui->movingSpeedSlider->setValue(mspeed);
    if (servoModel != SERVO_XL320)
    {
        int tlimit = servo->getTorqueLimit();
        ui->torqueLimitSlider->setValue(tlimit);
    }
    int cpos = servo->getCurrentPosition();
    ui->cpos->setValue(cpos);
    ui->cpos_label->setText(tr("Current (") + QString::number(cpos) + ")");

    double dcvolt = servo->getCurrentVoltage();
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
    ui->ctemp->display(ctemp);
}

/* ************************************************************************** */

void MainWindow::clearErrors()
{
    ControllerAPI *ctrl = nullptr;
    if (getCurrentController(ctrl) > 0)
    {
        ctrl->clearErrorCount();
    }

    Servo *servo = nullptr;
    if (getCurrentServo(servo) > 0)
    {
        servo->clearErrors();
    }

    ui->widgetStatus->handleErrors(ctrl, servo, 0, 0);
}

/* ************************************************************************** */

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
    ui->widgetStatus->setVisible(status);

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
        if (p->deviceController != nullptr)
        {
            if (!(p->deviceController->getState() == state_stopped ||
                  p->deviceController->getState() == state_scanned ||
                  p->deviceController->getState() == state_ready))
            {
                return;
            }
        }
    }

    if (asw == nullptr)
    {
        // Pause controllers
        for (auto p: serialPorts)
        {
            if (p->deviceController != nullptr)
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
    if (asw != nullptr)
    {
        // Un-pause controllers
        for (auto p: serialPorts)
        {
            if (p->deviceController != nullptr)
            {
                p->deviceController->pauseThread();
            }
        }

        // Un-lock current window
        this->setDisabled(false);

        // Delete the advance scanner window
        delete asw;
        asw = nullptr;
    }
}

/* ************************************************************************** */

void MainWindow::settingsStart()
{
    if (stw == nullptr)
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
    if (asw != nullptr)
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
    Servo *s = nullptr;

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
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        s->reboot();
    }
}

void MainWindow::refreshServo()
{
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        s->refresh();
    }
}

/* ************************************************************************** */

void MainWindow::toggleRunningMode()
{
    Servo *s = nullptr;

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
    Servo *s = nullptr;

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
    Servo *s = nullptr;

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
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveMaxTorque(#" << s->getId() << " / torque:" << torque << ")" << std::endl;

        // Update max torque
        if (currentServoSerie < SERVO_HERKULEX)
        {
            static_cast<ServoDynamixel*>(s)->setMaxTorque(torque);
        }
    }
}

void MainWindow::moveTorqueLimit(int limit)
{
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveTorqueLimit(#" << s->getId() << " / limit:" << limit << ")" << std::endl;

        // Update torque limit
        if (currentServoSerie == SERVO_XL)
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
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        //std::cout << "moveMovingSpeed(#" << s->getId() << " / moving speed:" << speed << ")" << std::endl;

        // Update moving speed
        if (currentServoSerie < SERVO_HERKULEX)
        {
            static_cast<ServoDynamixel*>(s)->setMovingSpeed(speed);
        }
    }
}

void MainWindow::moveCWLimit(int limit)
{
    Servo *s = nullptr;

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
    Servo *s = nullptr;

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
    Servo *s = nullptr;

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
    Servo *s = nullptr;

    // Get selected servo
    if (getCurrentServo(s) > 0)
    {
        int reg_value = ui->widgetRT->getRegisterValueFromTableIndex(row, column);
        int reg_name = ui->widgetRT->getRegisterNameFromTableIndex(currentServoSerie, currentServoModel, row);
        int reg_type = REGISTER_AUTO;

        if (currentServoSerie >= SERVO_HERKULEX)
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
                if (item != nullptr && item->parent() != nullptr)
                {
                    // Finaly change the item text
                    QString device_txt = "[#" + QString::number(reg_value) + "]  " + QString::fromStdString(s->getModelString());
                    item->setText(0, device_txt);
                }
            }
        }
    }
}
