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
 * \file SerialPortWindows.cpp
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#if defined(_WIN32) || defined(_WIN64)

#include "SerialPortWindows.h"
#include "minitraces.h"

// C++ standard library
#include <string>
#include <vector>

// Device lock support
#define LOCK_LOCKFILE

LPCWSTR stringToLPCWSTR(const std::string &s)
{
    int slength = static_cast<int>(s.length()) + 1;
    int len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t *buf = new wchar_t[len];

    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring temp(buf);
    delete[] buf;

    LPCWSTR l = temp.c_str();
    return l;
}

int serialPortsScanner(std::vector <std::string> &availableSerialPorts)
{
    int retcode = 0;
    std::string basePort = "\\\\.\\COM";

    TRACE_INFO(SERIAL, "serialPortsScanner() [Windows variant]");

    // Serial ports
    for (int i = 32; i > 0; i--)
    {
        std::string port = basePort + std::to_string(i);

#ifdef UNICODE
        HANDLE ghSerial_Handle = CreateFileW(stringToLPCWSTR(port), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
#else
        HANDLE ghSerial_Handle = CreateFileA(port.c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
#endif
        if (ghSerial_Handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(ghSerial_Handle);

            TRACE_1(SERIAL, "- Scanning for serial port on '%s' > FOUND", port.c_str());
            availableSerialPorts.push_back(port);
            retcode++;
        }
    }

    return retcode;
}

SerialPortWindows::SerialPortWindows(std::string &devicePath, const int baud, const int serialDevice, const int servoDevices):
    SerialPort(serialDevice, servoDevices),
    ttyDeviceFileDescriptor(0)
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
        size_t found = ttyDevicePath.rfind("\\");
        if (found != std::string::npos && found != ttyDevicePath.size())
        {
            ttyDeviceName = ttyDevicePath.substr(found + 1);

            // TODO
            //ttyDeviceLockPath = "/tmp/";
            //ttyDeviceLockPath += ttyDeviceName;
            //ttyDeviceLockPath += ".lock";
        }

        setBaudRate(baud);

        TRACE_INFO(SERIAL, "- Device name has been set to: '%s'", ttyDeviceName.c_str());
        TRACE_INFO(SERIAL, "- Device node has been set to: '%s'", ttyDevicePath.c_str());
        TRACE_INFO(SERIAL, "- Device baud rate has been set to: '%i'", ttyDeviceBaudRate);
    }
}

SerialPortWindows::~SerialPortWindows()
{
    closeLink();
}

void SerialPortWindows::setBaudRate(const int baud)
{
    // Get valid baud rate
    ttyDeviceBaudRate = checkBaudRate(baud);

    // Compute the time needed to transfert one byte through the serial interface
    // (1000 / baudrate(= bit per msec)) * 10(= start bit + 8 data bit + stop bit)
    byteTransfertTime = (1000.0 / static_cast<double>(ttyDeviceBaudRate)) * 10.0;
}

int SerialPortWindows::openLink()
{
    DCB Dcb;
    COMMTIMEOUTS Timeouts;
    DWORD dwError;

    // Make sure no tty connection is already running
    closeLink();

    // Check if another instance is using this port
    if (isLocked() == true)
    {
        TRACE_ERROR(SERIAL, "Cannot connect to serial port: '%s': interface is locked!", ttyDevicePath.c_str());
        goto OPEN_LINK_LOCKED;
    }

    // Open tty device
#ifdef UNICODE
    ttyDeviceFileDescriptor = CreateFileW(stringToLPCWSTR(ttyDeviceName), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
#else
    ttyDeviceFileDescriptor = CreateFileA(ttyDeviceName.c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
#endif
    if (ttyDeviceFileDescriptor == INVALID_HANDLE_VALUE)
    {
        TRACE_ERROR(SERIAL, "Unable to open device: '%s' error: '%i'", ttyDeviceName.c_str(), GetLastError());
        goto OPEN_LINK_ERROR;
    }

    // Lock device
    setLock();

    // Setting communication property
    Dcb.DCBlength = sizeof(DCB);
    if (GetCommState(ttyDeviceFileDescriptor, &Dcb) == FALSE)
    {
        TRACE_ERROR(SERIAL, "Unable to get communication state on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }

    // Set baudrate
    Dcb.BaudRate            = (DWORD)ttyDeviceBaudRate;
    Dcb.ByteSize            = 8;                    // Data bit = 8bit
    Dcb.Parity              = NOPARITY;             // No parity
    Dcb.StopBits            = ONESTOPBIT;           // Stop bit = 1
    Dcb.fParity             = NOPARITY;             // No Parity check
    Dcb.fBinary             = 1;                    // Binary mode
    Dcb.fNull               = 0;                    // Get Null byte
    Dcb.fAbortOnError       = 1;
    Dcb.fErrorChar          = 0;
    // Not using XOn/XOff
    Dcb.fOutX               = 0;
    Dcb.fInX                = 0;
    // Not using H/W flow control
    Dcb.fDtrControl         = DTR_CONTROL_DISABLE;
    Dcb.fRtsControl         = RTS_CONTROL_DISABLE;
    Dcb.fDsrSensitivity     = 0;
    Dcb.fOutxDsrFlow        = 0;
    Dcb.fOutxCtsFlow        = 0;

    if (SetCommState(ttyDeviceFileDescriptor, &Dcb) == FALSE)
    {
        TRACE_ERROR(SERIAL, "Unable to set communication state on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }
    if (SetCommMask(ttyDeviceFileDescriptor, 0) == FALSE) // Not using Comm event
    {
        TRACE_ERROR(SERIAL, "Unable to set communication mask on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }
    if (SetupComm(ttyDeviceFileDescriptor, 8192, 8192) == FALSE) // Buffer size (Rx,Tx)
    {
        TRACE_ERROR(SERIAL, "Unable to setup communication on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }
    if (PurgeComm(ttyDeviceFileDescriptor, PURGE_TXABORT|PURGE_TXCLEAR|PURGE_RXABORT|PURGE_RXCLEAR) == FALSE) // Clear buffer
    {
        TRACE_ERROR(SERIAL, "Unable to purge communication on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }
    if (ClearCommError(ttyDeviceFileDescriptor, &dwError, NULL) == FALSE)
    {
        TRACE_ERROR(SERIAL, "Unable to clear communication errors on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }
    if (GetCommTimeouts(ttyDeviceFileDescriptor, &Timeouts) == FALSE)
    {
        TRACE_ERROR(SERIAL, "Unable to get communication timeouts on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }

    // Timeout (Not using timeout)
    // Immediatly return
    Timeouts.ReadIntervalTimeout         = 0;
    Timeouts.ReadTotalTimeoutMultiplier  = 0;
    Timeouts.ReadTotalTimeoutConstant    = 1; // Must not be zero
    Timeouts.WriteTotalTimeoutMultiplier = 0;
    Timeouts.WriteTotalTimeoutConstant   = 0;

    if (SetCommTimeouts(ttyDeviceFileDescriptor, &Timeouts) == FALSE)
    {
        TRACE_ERROR(SERIAL, "Unable to set communication timeouts on '%s'", ttyDevicePath.c_str());
        goto OPEN_LINK_ERROR;
    }

    return 1;

OPEN_LINK_ERROR:
    closeLink();
    return 0;

OPEN_LINK_LOCKED:
    closeLink();
    return -1;
}

bool SerialPortWindows::isOpen()
{
    bool status = false;

    if (ttyDeviceFileDescriptor != INVALID_HANDLE_VALUE)
    {
        DCB Dcb;
        Dcb.DCBlength = sizeof(DCB);
        if (GetCommState(ttyDeviceFileDescriptor, &Dcb) == TRUE)
        {
            status = true;
        }
    }

    return status;
}

void SerialPortWindows::closeLink()
{
    if (isOpen())
    {
        this->flush();
        CloseHandle(ttyDeviceFileDescriptor);
        ttyDeviceFileDescriptor = INVALID_HANDLE_VALUE;

        removeLock();
    }
}

int SerialPortWindows::tx(unsigned char *packet, int packetLength)
{
    int status = -1;

    if (isOpen())
    {
        if (packet != nullptr && packetLength > 0)
        {
            DWORD dwToWrite = (DWORD)packetLength;
            DWORD dwWritten = 0;

            // Error handling
            DWORD dwError = 0;
            COMSTAT comstat;

            dwError = GetLastError();
            if (dwError)
            {
                TRACE_ERROR(SERIAL, "SerialPortWindows::tx(dwError: %i)", dwError);
                ClearCommError(ttyDeviceFileDescriptor, &dwError, &comstat);
            }

            // Send
            if (WriteFile(ttyDeviceFileDescriptor, packet, dwToWrite, &dwWritten, NULL) == TRUE)
            {
                status = static_cast<int>(dwWritten);
            }
            else
            {
                TRACE_ERROR(SERIAL, "Cannot write to serial port '%s': WriteFile() failed!", ttyDevicePath.c_str());
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

    return status;
}

int SerialPortWindows::rx(unsigned char *packet, int packetLength)
{
    int readStatus = -1;

    if (isOpen())
    {
        if (packet != nullptr && packetLength > 0)
        {
            DWORD dwToRead = (DWORD)packetLength;
            DWORD dwRead = 0;

            // Error handling
            DWORD dwError = 0;
            COMSTAT comstat;

            dwError = GetLastError();
            if (dwError)
            {
                TRACE_ERROR(SERIAL, "SerialPortWindows::rx(dwError: %i)", dwError);
                ClearCommError(ttyDeviceFileDescriptor, &dwError, &comstat);
            }

            // Receive
            if (ReadFile(ttyDeviceFileDescriptor, packet, dwToRead, &dwRead, NULL) == TRUE)
            {
                readStatus = static_cast<int>(dwRead);
            }
            else
            {
                TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': ReadFile() failed!", ttyDevicePath.c_str());
            }
        }
        else
        {
            TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': ReadFile() invalid packet buffer or size!", ttyDevicePath.c_str());
        }
    }
    else
    {
        TRACE_ERROR(SERIAL, "Cannot read from serial port '%s': invalid device!", ttyDevicePath.c_str());
    }

    return readStatus;
}

void SerialPortWindows::flush()
{
    if (isOpen())
    {
        //PURGE_RXABORT: Terminates all outstanding overlapped read operations and returns immediately, even if the read operations have not been completed.
        //PURGE_RXCLEAR: Clears the input buffer (if the device driver has one).
        //PURGE_TXABORT: Terminates all outstanding overlapped write operations and returns immediately, even if the write operations have not been completed.
        //PURGE_TXCLEAR: Clears the output buffer (if the device driver has one).

        PurgeComm(ttyDeviceFileDescriptor, PURGE_RXABORT | PURGE_RXCLEAR);
    }
}

double SerialPortWindows::getTime()
{
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);

    return static_cast<double>(time.QuadPart);
}

void SerialPortWindows::setTimeOut(int packetLength)
{
    packetStartTime = getTime();
    packetWaitTime  = (byteTransfertTime * static_cast<double>(packetLength) + 2.0 * static_cast<double>(ttyDeviceLatencyTime));
}

void SerialPortWindows::setTimeOut(double msec)
{
    packetStartTime = getTime();
    packetWaitTime  = msec;
}

int SerialPortWindows::checkTimeOut()
{
    LARGE_INTEGER end, freq;
    int status = 0;
    double time_elapsed = 0.0;

    QueryPerformanceCounter(&end);
    QueryPerformanceFrequency(&freq);

    time_elapsed = static_cast<double>(end.QuadPart - packetStartTime) / static_cast<double>(freq.QuadPart);
    time_elapsed *= 1000.0;

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

#endif // _WIN32 || _WIN64
