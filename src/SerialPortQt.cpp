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
 * \file SerialPortQt.cpp
 * \date 28/08/2016
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#if defined(FEATURE_QTSERIAL)

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 7, 0)

#include "SerialPortQt.h"
#include "minitraces.h"

// C++ standard libraries
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <thread>

#include <QSerialPortInfo>
#include <QDebug>

int serialPortsScannerQt(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;

    TRACE_INFO(SERIAL, "serialPortsScanner() [QtSerialPort variant]");

    QSerialPortInfo spi;

    QList<QSerialPortInfo> asp = spi.availablePorts();

    qDebug() << "Available serial ports: ";
    for (QSerialPortInfo sp: asp)
    {
        qDebug() << "> Port #" << retcode + 1;
        qDebug() << "- systemLocation: " << sp.systemLocation();
        qDebug() << "- desc: " << sp.description();
        qDebug() << "- port name: " << sp.portName();
        qDebug() << "- manufacturer: " << sp.manufacturer();
        qDebug() << "- standard BaudRates: " << sp.standardBaudRates();

        TRACE_INFO(SERIAL, "> Scanning for serial port on '%s' > FOUND", sp.systemLocation().toLocal8Bit().data());
        availableSerialPorts.push_back(sp.systemLocation().toLocal8Bit().data());
        retcode++;
    }

    return retcode;
}

SerialPortQt::SerialPortQt(std::string &devicePath, const int baud, const int serialDevice, const int servoDevices):
    SerialPort(serialDevice, servoDevices)
{
    if (devicePath.empty() == 1 || devicePath == "auto")
    {
        ttyDevicePath = autoselectSerialPort();
    }
    else
    {
        ttyDevicePath = devicePath;
    }

    if (ttyDevicePath != "null")
    {
        size_t found = ttyDevicePath.rfind("/");
        if (found != std::string::npos && found != ttyDevicePath.size())
        {
            ttyDeviceName = ttyDevicePath.substr(found + 1);

            //ttyDeviceLockPath = "/var/lock/lockdev/LCK..";
            ttyDeviceLockPath = "/tmp/LCK..";
            ttyDeviceLockPath += ttyDeviceName;
        }

        setBaudRate(baud);

        TRACE_INFO(SERIAL, "- Device name has been set to: '%s'", ttyDeviceName.c_str());
        TRACE_INFO(SERIAL, "- Device node has been set to: '%s'", ttyDevicePath.c_str());
        TRACE_INFO(SERIAL, "- Device baud rate has been set to: '%i'", ttyDeviceBaudRate);
    }

    serial = new QSerialPort();
    QString lockfilename = "SmartServoFramework";
    lock = new QLockFile(lockfilename);
}

SerialPortQt::~SerialPortQt()
{
    delete serial;
    serial = nullptr;

    delete lock;
    lock = nullptr;

    closeLink();
}

void SerialPortQt::setBaudRate(const int baud)
{
    // Get valid baud rate
    ttyDeviceBaudRate = checkBaudRate(baud);

    // Get <termios.h> baudrate flag
    //ttyDeviceBaudRateFlag = convertBaudRateFlag(ttyDeviceBaudRate);

    // Compute the time needed to transfert one byte through the serial interface
    // (1000 / baudrate(= bit per msec)) * 10(= start bit + 8 data bit + stop bit)
    byteTransfertTime = (1000.0 / static_cast<double>(ttyDeviceBaudRate)) * 10.0;
}

int SerialPortQt::convertBaudRateFlag(int baudrate)
{
    int baudRateFlag = 0;

    // Set termios baudrate flag
    if (baudrate > 0)
    {
        // Try an exact match
        baudRateFlag = 0;

        if (baudRateFlag != 0)
        {
            TRACE_1(SERIAL, "convertBaudRateFlag(%i) has been set to %i (exact match)", baudrate, baudrate);
        }
        else
        {
            int speeds[20] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
                              460800, 500000, 576000, 921600, 1000000, 1152000,
                              1500000, 2000000, 2500000, 3000000, 3500000, 4000000};

            // Try a "close enough" match (we allow ±1.5% mismatch)
            for (int i = 0; i < 20; i++)
            {
                if ((baudrate > (static_cast<double>(speeds[i]) * 98.5 / 100.0)) && (baudrate < (static_cast<double>(speeds[i]) * 101.5 / 100.0)))
                {
                    //baudRateFlag = rate_to_constant(speeds[i]);
                    TRACE_WARNING(SERIAL, "convertBaudRateFlag(%i) has been set to B%i (close enough match, ±1.5%)", baudrate, speeds[i]);
                    break;
                }
            }

            // Try a custom speed
            if (baudRateFlag == 0)
            {
                //ttyCustomSpeed = true;
                //baudRateFlag = B38400;
                TRACE_WARNING(SERIAL, "convertBaudRateFlag(%i) has been set to B38400 (custom speed will be used)", baudrate);
            }
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Invalid baudrate, using default value of: B1000000");
    }

    // Fallback
    if (baudRateFlag == 0)
    {
        //baudRateFlag = B1000000;
        TRACE_ERROR(SERIAL, "Unable to set baud speed at %i: too slow!", baudrate);
        TRACE_ERROR(SERIAL, "Invalid baudrate, using default value of: B1000000");
    }

    return baudRateFlag;
}

bool SerialPortQt::isLocked()
{
    bool status = lock->isLocked();

    return status;
}

bool SerialPortQt::setLock()
{
    bool status = false;

    // First, check if device locking has been enabled for this link
    if (ttyDeviceLockMode > 0)
    {
        status = lock->tryLock();

        if (status == true)
        {
            lock->setStaleLockTime(24 * 3600 * 1000);
        }
    }

    return status;
}

bool SerialPortQt::removeLock()
{
    bool status = false;

    lock->unlock();

    if (lock->isLocked() == false)
    {
        status = true;
    }
    else
    {
        bool force = false;
        if (force == true)
        {
            status == lock->removeStaleLockFile();
        }
    }

    return true;
}

int SerialPortQt::openLink()
{
    // Make sure no tty connection is already running (in that case, openLink() will do a reconnection)
    closeLink();

    // Check if another instance is using this port
    if (isLocked() == true)
    {
        TRACE_ERROR(SERIAL, "Cannot connect to serial port: '%s': interface is locked!", ttyDevicePath.c_str());
        goto OPEN_LINK_LOCKED;
    }

    // Open tty device
    serial->setPortName(QString::fromStdString(ttyDeviceName));
    serial->setBaudRate(ttyDeviceBaudRate);

    if (serial->open(QIODevice::ReadWrite) == false)
    {
        TRACE_ERROR(SERIAL, "Unable to open device on serial port: '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }

    // Lock device
    setLock();

    // Set newtio attributes

    if (ttyDeviceBaudRate < 1)
    {
        TRACE_ERROR(SERIAL, "Unable to set baud rate to '%i'bps: invalid value", ttyDeviceBaudRate);
        goto OPEN_LINK_ERROR;
    }

    // Set custom serial infos?

    return 1;

OPEN_LINK_ERROR:
    closeLink();
    return 0;

OPEN_LINK_LOCKED:
    closeLink();
    return -1;
}

bool SerialPortQt::isOpen()
{
    bool status = false;

    if (serial->openMode() == QIODevice::ReadWrite)
    {
        status = true;
    }

    return status;
}

void SerialPortQt::closeLink()
{
    if (isOpen() == true)
    {
        this->flush();
        serial->close();

        removeLock();
    }
}

int SerialPortQt::tx(unsigned char *packet, int packetLength)
{
    int writeStatus = -1;

    if (isOpen() == true)
    {
        if (packet != nullptr && packetLength > 0)
        {
            writeStatus = serial->write((char *)packet, packetLength);
            //writeStatus = write(ttyDeviceFileDescriptor, packet, packetLength);

            if (writeStatus < 0)
            {
                TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': write() failed with error code '%i'!", ttyDevicePath.c_str(), errno);
            }
        }
        else
        {
            TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': invalid packet buffer or size!", ttyDevicePath.c_str());
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': invalid device!", ttyDevicePath.c_str());
    }

    return writeStatus;
}

int SerialPortQt::rx(unsigned char *packet, int packetLength)
{
    int readStatus = -1;

    if (isOpen() == true)
    {
        if (packet != nullptr && packetLength > 0)
        {
            memset(packet, 0, packetLength);
            readStatus = serial->read((char *)packet, packetLength);
            //readStatus = read(ttyDeviceFileDescriptor, packet, packetLength);

            if (readStatus < 0)
            {
                TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': read() failed with error code '%i'!", ttyDevicePath.c_str(), errno);
            }
        }
        else
        {
            TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': invalid packet buffer or size!", ttyDevicePath.c_str());
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': invalid device!", ttyDevicePath.c_str());
    }

    return readStatus;
}

void SerialPortQt::flush()
{
    if (isOpen() == true)
    {
        serial->flush();
    }
}

double SerialPortQt::getTime()
{
    //struct timeval tv;
    //gettimeofday(&tv, nullptr);

    return 0;//(static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0);
}

bool SerialPortQt::switchHighSpeed()
{
    bool status = false;

    // TODO // enables "ASYNC_LOW_LATENCY" flag
    // reduce the "/sys/bus/usb-serial/devices/ttyXXX/latency_timer" value

    return status;
}

void SerialPortQt::setLatency(int latency)
{
    // "auto-detect" latency value if possible
    // note: this doesn't really produce expected results yet..
    if (latency == 0)
    {
        size_t lastbackslash = ttyDevicePath.find_last_of('/');

        if (lastbackslash != std::string::npos)
        {
            std::string latency_path = "/sys/bus/usb-serial/devices/" + ttyDevicePath.substr(lastbackslash+1) + "/latency_timer";
            std::ifstream latency_file(latency_path);

            if (latency_file.good())
            {
                char latency_value[8];
                latency_file.getline(latency_value, 8);
                latency = std::stoi(latency_value);
            }
            latency_file.close();
        }
        else
        {
            TRACE_ERROR(SERIAL, "Unable to find latency info for the current device: '%s'", ttyDevicePath.substr(lastbackslash+1).c_str());
        }
    }

    if (latency > 0 && latency < 128)
    {
        ttyDeviceLatencyTime = latency;
    }
    else
    {
        TRACE_WARNING(SERIAL, "Invalid latency value: '%i', not in ]0;128[ range.", latency);
    }
}

void SerialPortQt::setTimeOut(int packetLength)
{
    packetStartTime = getTime();
    packetWaitTime  = (byteTransfertTime * static_cast<double>(packetLength) + 2.0 * static_cast<double>(ttyDeviceLatencyTime));
}

void SerialPortQt::setTimeOut(double msec)
{
    packetStartTime = getTime();
    packetWaitTime  = msec;
}

int SerialPortQt::checkTimeOut()
{
    int status = 0;
    double time_elapsed = getTime() - packetStartTime;

    if (time_elapsed > packetWaitTime)
    {
        status = 1;
    }
    else if (time_elapsed < 0)
    {
        packetStartTime = getTime();
    }

    return status;
}

#endif // QT_VERSION >= QT_VERSION_CHECK(5, 7, 0)
#endif // FEATURE_QTSERIAL
