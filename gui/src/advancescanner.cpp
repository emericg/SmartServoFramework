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
 * \file AdvanceScanner.cpp
 * \date 5/09/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "advancescanner.h"
#include "ui_advancescanner.h"
#include "mainwindow.h"

#include <iostream>
#include <QCloseEvent>

AdvanceScanner::AdvanceScanner(QMainWindow *main, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AdvanceScanner)
{
    this->parent = main;
    ui->setupUi(this);

    ui->label_scan_errors->setVisible(false);
    ui->actionStartScan->setEnabled(false);
    ui->actionStopScan->setEnabled(false);
    ui->frame_scan->hide();

    QObject::connect(ui->actionStartScan, SIGNAL(triggered()), this, SLOT(startScan()));
    QObject::connect(ui->actionStopScan, SIGNAL(triggered()), this, SLOT(stopScan()));
    QObject::connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(exitScan()));

    QObject::connect(ui->pushButton_ports, SIGNAL(clicked()), this, SLOT(fillWidgets_ports()));
    QObject::connect(ui->pushButton_quickscan, SIGNAL(clicked()), this, SLOT(quickProfile()));
    QObject::connect(ui->pushButton_fullscan, SIGNAL(clicked()), this, SLOT(fullProfile()));

    QObject::connect(this, SIGNAL(exiting()), this->parent, SLOT(advanceScannerStop()));

    fillWidgets_ports();
    fillWidgets_speeds();
    quickProfile();

#ifdef Q_OS_OSX
    ui->toolBar->setStyleSheet("");
#endif
}

AdvanceScanner::~AdvanceScanner()
{
    delete ui;
}

void AdvanceScanner::fillWidgets_ports()
{
    // Cleanup
    for (int i = ui->tableWidget_ports->rowCount(); i >= 0; i--)
    {
        ui->tableWidget_ports->removeRow(i);
    }

    // Scan for available serial ports
    std::vector <std::string> availablePorts;
    serialPortsScanner(availablePorts);

    if (availablePorts.size() == 0)
    {
        QTableWidgetItem *port = new QTableWidgetItem("No serial port available!");
        port->setFlags(port->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        ui->tableWidget_ports->insertRow(0);
        ui->tableWidget_ports->setItem(0, 0, port);
    }
    else
    {
        ui->actionStartScan->setEnabled(true);

        int row = 0;
        for (auto p: availablePorts)
        {
            QTableWidgetItem *port = new QTableWidgetItem(QString::fromStdString(p));
            port->setFlags(port->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
            port->setCheckState(Qt::Checked);

            ui->tableWidget_ports->insertRow(row);
            ui->tableWidget_ports->setItem(row, 0, port);
            row++;
        }
    }
}

void AdvanceScanner::fillWidgets_speeds()
{
    int row = 0;
/*
    // Cleanup
    for (int i = ui->tableWidget_speed_dxl_v1->rowCount(); i >= 0; i--)
    {
        ui->tableWidget_speed_dxl_v1->removeRow(i);
    }
    for (int i = ui->tableWidget_speed_dxl_v2->rowCount(); i >= 0; i--)
    {
        ui->tableWidget_speed_dxl_v2->removeRow(i);
    }
    for (int i = ui->tableWidget_speed_hkx->rowCount(); i >= 0; i--)
    {
        ui->tableWidget_speed_hkx->removeRow(i);
    }
    ui->treeWidget_results->clear();
*/

/*
    // DXL v1
    int row = 0;
    //for (int i = 249; i < 253; i++)
    //{
    //    QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(i, SERVO_MX)));
    //    rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
    //    rate->setCheckState(Qt::Checked);
    //    ui->tableWidget_speed_dxl_v1->insertRow(row);
    //    ui->tableWidget_speed_dxl_v1->setItem(row, 0, rate);
    //    row++;
    //}
    for (int i = 1; i < 100; i++)
    {
        QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(i)));
        rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        rate->setCheckState(Qt::Checked);
        ui->tableWidget_speed_dxl_v1->insertRow(row);
        ui->tableWidget_speed_dxl_v1->setItem(row, 0, rate);
        row++;
    }
*/
    // DXL v2
    row = 0;
    for (int i = 0; i < 8; i++)
    {
        QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(i, SERVO_X)));
        rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        rate->setCheckState(Qt::Checked);
        ui->tableWidget_speed_dxl_v2->insertRow(row);
        ui->tableWidget_speed_dxl_v2->setItem(row, 0, rate);
        row++;
    }
/*
    // DXL PRO
    for (int i = 1; i < 9; i++)
    {
        QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(i, SERVO_PRO)));
        rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        rate->setCheckState(Qt::Checked);
        ui->tableWidget_speed_dxl_v2->insertRow(row);
        ui->tableWidget_speed_dxl_v2->setItem(row, 0, rate);
        row++;
    }
*/
    // HKX
    row = 0;
    int baudnums_hkx[8] = {0x01, 0x02, 0x03, 0x04, 0x07, 0x09, 0x10, 0x22};
    for (int i = 0; i < 8; i++)
    {
        QTableWidgetItem *rate = new QTableWidgetItem(QString::number(hkx_get_baudrate(baudnums_hkx[i])));
        rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
        rate->setCheckState(Qt::Checked);
        ui->tableWidget_speed_hkx->insertRow(row);
        ui->tableWidget_speed_hkx->setItem(row, 0, rate);
        row++;
    }
}

void AdvanceScanner::quickProfile()
{
    setProfil(0);
}

void AdvanceScanner::fullProfile()
{
    setProfil(1);
}

void AdvanceScanner::setProfil(int profile)
{
    if (profile == 0) // "Quick scan"
    {
        ui->checkBox_speed_quickstop->setChecked(true);

        // Clean the DXL v1 box
        for (int i = ui->tableWidget_speed_dxl_v1->rowCount(); i >= 0; i--)
        {
            ui->tableWidget_speed_dxl_v1->removeRow(i);
        }
        // Fill it again
        int selectionDXLv1[4] = {1, 2, 16, 34};
        int row = 0;
        for (int i = 0; i < 4; i++)
        {
            QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(selectionDXLv1[i])));
            rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
            rate->setCheckState(Qt::Checked);
            ui->tableWidget_speed_dxl_v1->insertRow(row);
            ui->tableWidget_speed_dxl_v1->setItem(row, 0, rate);
            row++;
        }

        // Check all the DXL v2 boxes!
        for (int i = 0; i < ui->tableWidget_speed_dxl_v2->rowCount(); i++)
        {
            ui->tableWidget_speed_dxl_v2->item(i, 0)->setCheckState(Qt::Checked);
        }

        // Uncheck all the HKX boxes
        for (int i = 0; i < ui->tableWidget_speed_hkx->rowCount(); i++)
        {
            ui->tableWidget_speed_hkx->item(i, 0)->setCheckState(Qt::Unchecked);
        }
        // Then set custom selection
        int selectionHKX[3] = {0, 6, 7};

        for (int i = 0; i < 3; i++)
        {
            ui->tableWidget_speed_hkx->item(selectionHKX[i], 0)->setCheckState(Qt::Checked);
        }
    }
    else // if (profile == 1) // "Full scan"
    {
        ui->checkBox_speed_quickstop->setChecked(false);

        // Clean the DXL v1 box
        for (int i = ui->tableWidget_speed_dxl_v1->rowCount(); i >= 0; i--)
        {
            ui->tableWidget_speed_dxl_v1->removeRow(i);
        }
        // Fill it again
        int row = 0;
        for (int i = 1; i < 200; i++)
        {
            QTableWidgetItem *rate = new QTableWidgetItem(QString::number(dxl_get_baudrate(i)));
            rate->setFlags(rate->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
            rate->setCheckState(Qt::Checked);
            ui->tableWidget_speed_dxl_v1->insertRow(row);
            ui->tableWidget_speed_dxl_v1->setItem(row, 0, rate);
            row++;
        }

        // Check all the boxes!
        for (int i = 0; i < ui->tableWidget_speed_dxl_v2->rowCount(); i++)
        {
            ui->tableWidget_speed_dxl_v2->item(i, 0)->setCheckState(Qt::Checked);
        }
        for (int i = 0; i < ui->tableWidget_speed_hkx->rowCount(); i++)
        {
            ui->tableWidget_speed_hkx->item(i, 0)->setCheckState(Qt::Checked);
        }
    }
}

void AdvanceScanner::resizeEvent(QResizeEvent *event)
{
    resizeTables();
}

void AdvanceScanner::closeEvent(QCloseEvent *event)
{
    // If a scanning process is underway, it will need to stop itself before closing the window
    if (scan_running == true)
    {
        event->ignore();
    }

    exitScan();
}

void AdvanceScanner::resizeTables()
{
    // Register tables
    int width_total = ui->tableWidget_ports->size().width();
    int height_total = ((ui->tableWidget_ports->rowHeight(0) + 1) * ui->tableWidget_ports->rowCount()) + ui->tableWidget_ports->horizontalHeader()->size().height();

    ui->tableWidget_ports->setColumnWidth(0, width_total);
    ui->tableWidget_ports->setMaximumHeight(height_total);

    ui->tableWidget_speed_dxl_v1->setColumnWidth(0, ui->tableWidget_speed_dxl_v1->size().width());
    ui->tableWidget_speed_dxl_v2->setColumnWidth(0, ui->tableWidget_speed_dxl_v2->size().width());
    ui->tableWidget_speed_hkx->setColumnWidth(0, ui->tableWidget_speed_hkx->size().width());
}

void AdvanceScanner::startScan()
{
    // Check if the scanning process is already running
    if (scan_running == true)
    {
        return;
    }

    // Mark scan as started
    scan_running = true;
    ui->frame_scan->show();

    // Lock GUI
    ui->actionStopScan->setEnabled(true);
    ui->groupBox_serialports->setEnabled(false);
    ui->groupBox_IDs->setEnabled(false);
    ui->groupBox_protocols->setEnabled(false);
    ui->groupBox_speeds->setEnabled(false);
    ui->label_scanning->setText(tr("Scanning (in progress)"));

    // Init progress bars
    progress_global = 0.0;
    progress_current = 0.0;

    // Clean results
    ui->treeWidget_results->clear();
    ui->treeWidget_results->update();
    ui->label_scan_errors->hide();

    // Count scan rounds and interval
    scans_rounds = 0;
    scans_id_interval = ui->spinBox_maxid->value() - ui->spinBox_minid->value();
    for (int i = 0; i < ui->tableWidget_ports->rowCount(); i++)
    {
        if (ui->tableWidget_ports->item(i, 0)->checkState() == Qt::Checked)
        {
            if (ui->checkBox_proto_dxl_v1->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_dxl_v1->rowCount(); j++)
                {
                    if (ui->tableWidget_speed_dxl_v1->item(j, 0)->checkState() == Qt::Checked)
                    {
                        scans_rounds++;
                    }
                }
            }
            if (ui->checkBox_proto_dxl_v2->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_dxl_v2->rowCount(); j++)
                {
                    if (ui->tableWidget_speed_dxl_v2->item(j, 0)->checkState() == Qt::Checked)
                    {
                        scans_rounds++;
                    }
                }
            }
            if (ui->checkBox_proto_hkx->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_hkx->rowCount(); j++)
                {
                    if (ui->tableWidget_speed_hkx->item(j, 0)->checkState() == Qt::Checked)
                    {
                        scans_rounds++;
                    }
                }
            }
        }
    }

    // Scan each ports
    for (int i = 0; i < ui->tableWidget_ports->rowCount(); i++)
    {
        int results = 0;

        if (ui->tableWidget_ports->item(i, 0)->checkState() == Qt::Checked)
        {
            std::string port = ui->tableWidget_ports->item(i, 0)->text().toStdString();

            // Add serial port to the UI
            QTreeWidgetItem *wport = new QTreeWidgetItem();
            ui->treeWidget_results->addTopLevelItem(wport);
            wport->setText(0, QString::fromStdString(port));
            wport->setExpanded(true);
            ui->treeWidget_results->update();

            // DXL v1
            if (ui->checkBox_proto_dxl_v1->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_dxl_v1->rowCount(); j++)
                {
                    // Scanning process aborted or "quickstop" hit ?!
                    if (scan_running == false ||
                        (results > 0 && ui->checkBox_speed_quickstop->isChecked() == true))
                    {
                        break;
                    }

                    // Process events to keep GUI interactive
                    qApp->processEvents();

                    // For each speed selected
                    if (ui->tableWidget_speed_dxl_v1->item(j, 0)->checkState() == Qt::Checked)
                    {
                        int baudnum = j+1;
                        int selectionDXLv1[4] = {1, 2, 16, 34};
                        if (ui->tableWidget_speed_dxl_v1->rowCount() == 4)
                        {
                            baudnum = selectionDXLv1[j];
                        }
                        int baudrate = dxl_get_baudrate(baudnum, SERVO_MX);

                        // Initialize a Dynamixel (v1) instance on current serial port
                        DynamixelSimpleAPI dxl(SERVO_MX);
                        if (dxl.connect(port, baudrate) != 0)
                        {
                            ui->treeWidget_results->setCurrentItem(wport);

                            // Update progress string
                            ui->label_progress->setText("Dynamixel v1  on  " + QString::fromStdString(port) + "  @  " + QString::number(baudrate));

                            // Scan serial port for servos, store results inside a vector
                            results += servoscan_dxl(&dxl, wport, baudrate);

                            // Close device(s)
                            dxl.disconnect();
                        }
                    }
                }
            }

            // DXL v2
            if (ui->checkBox_proto_dxl_v2->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_dxl_v2->rowCount(); j++)
                {
                    // Scanning process aborted or "quickstop" hit ?!
                    if (scan_running == false ||
                        (results > 0 && ui->checkBox_speed_quickstop->isChecked() == true))
                    {
                        break;
                    }

                    // Process events to keep GUI interactive
                    qApp->processEvents();

                    if (ui->tableWidget_speed_dxl_v2->item(j, 0)->checkState() == Qt::Checked)
                    {
                        int baudnum = j;
                        int baudrate = dxl_get_baudrate(baudnum, SERVO_X);

                        // Initialize a Dynamixel (v2) instance on current serial port
                        DynamixelSimpleAPI dxl(SERVO_X);
                        if (dxl.connect(port, baudrate) != 0)
                        {
                            // Update progress string
                            ui->label_progress->setText("Dynamixel v2  on  " + QString::fromStdString(port) + "  @  " + QString::number(baudrate));

                            // Scan serial port for servos, store results inside a vector
                            results += servoscan_dxl(&dxl, wport, baudrate);

                            // Close device(s)
                            dxl.disconnect();
                        }
                    }
                }
            }

            // HKX
            if (ui->checkBox_proto_hkx->isChecked() == true)
            {
                for (int j = 0; j < ui->tableWidget_speed_hkx->rowCount(); j++)
                {
                    // Scanning process aborted or "quickstop" hit ?!
                    if (scan_running == false ||
                        (results > 0 && ui->checkBox_speed_quickstop->isChecked() == true))
                    {
                        break;
                    }

                    // Process events to keep GUI interactive
                    qApp->processEvents();

                    if (ui->tableWidget_speed_hkx->item(j, 0)->checkState() == Qt::Checked)
                    {
                        int baudnum = j;
                        int baudnums_hkx[8] = {0x01, 0x02, 0x03, 0x04, 0x07, 0x09, 0x10, 0x22};
                        int baudrate = hkx_get_baudrate(baudnums_hkx[baudnum]);

                        // Initialize a HerkuleX instance on current serial port
                        HerkuleXSimpleAPI hkx;
                        if (hkx.connect(port, baudrate) != 0)
                        {
                            // Update progress string
                            ui->label_progress->setText("HerkuleX  on  " + QString::fromStdString(port) + "  @  " + QString::number(baudrate));

                            // Scan serial port for servos, store results inside a vector
                            results += servoscan_hkx(&hkx, wport, baudrate);

                            // Close device(s)
                            hkx.disconnect();
                        }
                    }
                }
            }

            // No servo for this port? Say it
            if (wport->childCount() == 0)
            {
                QTreeWidgetItem *nochild = new QTreeWidgetItem();
                wport->addChild(nochild);
                nochild->setText(0, tr("No devices found!"));
            }
        }
    }

    // Check if the scanning process has been aborted
    if (exit_programmed == true)
    {
        emit exiting();
        return;
    }

    // Scan is stopped
    scan_running = false;

    // Unlock GUI
    ui->actionStopScan->setEnabled(false);
    ui->groupBox_serialports->setEnabled(true);
    ui->groupBox_IDs->setEnabled(true);
    ui->groupBox_protocols->setEnabled(true);
    ui->groupBox_speeds->setEnabled(true);
    ui->label_scanning->setText(tr("Scanning completed!"));
    ui->label_progress->setText("");
    ui->progressBar_current->setValue(100);
/*
    // TODO // Handle errors?
    if (ui->progressBar_global->value() != 100)
    {
        if (ui->pushButton_quickscan->isChecked() == false && results = 0)
        {
            ui->label_scan_errors->show();
        }
    }
*/
}

void AdvanceScanner::stopScan()
{
    // Check if the scanning process is already running
    if (scan_running == true)
    {
        // Stop scan
        scan_running = false;
    }
}

void AdvanceScanner::exitScan()
{
    // Check if a scanning process is running
    // It will need to stop itself before closing the window
    if (scan_running == true)
    {
        scan_running = false;
        exit_programmed = true;
        return;
    }

    // Otherwise just exit and destroy this window
    emit exiting();
}

int AdvanceScanner::servoscan_dxl(DynamixelSimpleAPI *dxl, QTreeWidgetItem *port, int baudrate)
{
    int results = 0;
    int start = ui->spinBox_minid->value();
    int stop = ui->spinBox_maxid->value();
    QTreeWidgetItem *speed = nullptr;

    // Check start/stop boundaries
    if (start < 0 || start > (254 - 1))
        start = 0;

    if (stop < 1 || stop > 254 || stop < start)
        stop = 254;

    // Set shorter timeout value to dramatically decrease scanning time
#if defined(_WIN32) || defined(_WIN64)
    dxl->serialSetLatency(12);
#else
    dxl->serialSetLatency(8);
#endif

    std::cout << "> Scanning for Dynamixel devices on '" << dxl->serialGetCurrentDevice() << "'... Range is [" << start << "," << stop << "]" << std::endl;

    // Scan every IDs
    for (int id = start; id <= stop; id++)
    {
        // Check if the scanning process has been aborted
        if (scan_running == false || exit_programmed == true)
        {
            break;
        }

        // Process events to keep GUI interactive
        qApp->processEvents();

        // If the ping gets a response, then we have found a servo
        PingResponse pingstats;
        if (dxl->ping(id, &pingstats) == true)
        {
            dxl->setLed(id, 1, LED_GREEN);

            std::string model = dxl_get_model_name(pingstats.model_number);
            //std::cout << "[#" << id << "] DYNAMIXEL servo found!" << std::endl;
            //std::cout << "[#" << id << "] model: " << pingstats.model_number << " (" << model << ")" << std::endl;

            if (speed == nullptr)
            {
                // Add it to the interface
                speed = new QTreeWidgetItem();
                port->addChild(speed);
                speed->setText(0, QString::number(baudrate));
                speed->setExpanded(true);
            }

            // Add it to the interface
            QTreeWidgetItem *servo = new QTreeWidgetItem();
            servo->setText(0, "[#" + QString::number(id) + "]   " + QString::fromStdString(model));
            servo->setIcon(0, QIcon(":/devices/devices/DXL.svg"));
            speed->addChild(servo);
            ui->treeWidget_results->update();
            results++;

            dxl->setLed(id, 0);
        }

        // Update progressbar
        progressbar();
    }

    // Restore default timeout value
    dxl->serialSetLatency(LATENCY_TIME_DEFAULT);

    std::cout << std::endl;
    return results;
}

int AdvanceScanner::servoscan_hkx(HerkuleXSimpleAPI *hkx, QTreeWidgetItem *port, int baudrate)
{
    int results = 0;
    int start = ui->spinBox_minid->value();
    int stop = ui->spinBox_maxid->value();
    QTreeWidgetItem *speed = nullptr;

    // Check start/stop boundaries
    if (start < 0 || start > (254 - 1))
        start = 0;

    if (stop < 1 || stop > 254 || stop < start)
        stop = 254;

    // Set shorter timeout value to dramatically decrease scanning time
#if defined(_WIN32) || defined(_WIN64)
    hkx->serialSetLatency(12);
#else
    hkx->serialSetLatency(8);
#endif

    std::cout << "> Scanning for HerkuleX devices on '" << hkx->serialGetCurrentDevice() << "'... Range is [" << start << "," << stop << "]" << std::endl;

    for (int id = start; id <= stop; id++)
    {
        // Check if the scanning process has been aborted
        if (scan_running == false || exit_programmed == true)
        {
            break;
        }

        // Process events to keep GUI interactive
        qApp->processEvents();

        // If the ping gets a response, then we have found a servo
        PingResponse pingstats;
        if (hkx->ping(id, &pingstats) == true)
        {
            hkx->setLed(id, 1, LED_GREEN);

            std::string model = hkx_get_model_name(pingstats.model_number);
            //std::cout << "[#" << id << "] HerkuleX servo found!" << std::endl;
            //std::cout << "[#" << id << "] model: " << pingstats.model_number << " (" << model << ")" << std::endl;

            if (speed == nullptr)
            {
                // Add it to the interface
                speed = new QTreeWidgetItem();
                port->addChild(speed);
                speed->setText(0, QString::number(baudrate));
                speed->setExpanded(true);
            }

            // Add it to the interface
            QTreeWidgetItem *servo = new QTreeWidgetItem();
            servo->setText(0, "[#" + QString::number(id) + "]   " + QString::fromStdString(model));
            servo->setIcon(0, QIcon(":/devices/devices/HKX.svg"));
            speed->addChild(servo);
            ui->treeWidget_results->update();
            results++;

            hkx->setLed(id, 0);
        }

        // Update progressbar
        progressbar();
    }

    // Restore default timeout value
    hkx->serialSetLatency(LATENCY_TIME_DEFAULT);

    std::cout << std::endl;
    return results;
}

void AdvanceScanner::progressbar()
{
    if (scan_running == true)
    {
        progress_current += (100.0 / scans_id_interval);
/*
        std::cout << "Progress current: " << progress_current << std::endl;
        std::cout << "Progress global: " << progress_global << std::endl;
        std::cout << "Scan rounds: " << scans_rounds << " so this means: " << (100.0 / scans_rounds) << "%" << std::endl;
*/
        if (progress_current >= 100)
        {
            progress_current = 0;
            progress_global += (100.0 / scans_rounds);
        }

        ui->progressBar_current->setValue(progress_current);
        ui->progressBar_global->setValue(progress_global);
    }
}
