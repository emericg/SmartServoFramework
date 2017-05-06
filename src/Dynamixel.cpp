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
 * \file Dynamixel.cpp
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "Dynamixel.h"
#include "minitraces.h"

// C++ standard libraries
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>

/* ************************************************************************** */

// Enable latency timer
//#define LATENCY_TIMER

// Enable packet debugger
//#define PACKET_DEBUGGER

/* ************************************************************************** */

/*!
 * \brief The different instructions available with version 1 of the Dynamixel protocol.
 */
enum DynamixelProtocolV1 {
    INST_PING           = 1,
    INST_READ           = 2,
    INST_WRITE          = 3,
    INST_REG_WRITE      = 4,
    INST_ACTION         = 5,
    INST_FACTORY_RESET  = 6,
    INST_SYNC_WRITE     = 131  // 0x83
};

/*!
 * \brief The different instructions available with version 2 of the Dynamixel protocol.
 */
enum DynamixelProtocolV2 {
    INST_REBOOT         = 8,
    INST_STATUS         = 85,  // 0x55
    INST_SYNC_READ      = 130, // 0x82
    INST_BULK_READ      = 146, // 0x92
    INST_BULK_WRITE     = 147  // 0x93
};

/*!
 * \brief Addresses of the various fields forming a packet.
 * http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
 */
enum {
    PKT1_HEADER0        = 0,            //!< "0xFF". The 2 bytes header indicate the beginning of a packet.
    PKT1_HEADER1        = 1,            //!< "0xFF"
    PKT1_ID             = 2,            //!< It is the ID of Dynamixel device which will receive the Instruction Packet. Range is [0;254].
    PKT1_LENGTH         = 3,            //!< Length of the packet after this field (number of parameters + 2).
    PKT1_INSTRUCTION    = 4,
    PKT1_ERRBIT         = 4,
    PKT1_PARAMETER      = 5
};

/*!
 * \brief Addresses of the various fields forming a packet.
 * http://support.robotis.com/en/product/dynamixel_pro/communication.htm
 */
enum {
    PKT2_HEADER0        = 0,            //!< "0xFF". The 3 bytes header indicate the beginning of a packet.
    PKT2_HEADER1        = 1,            //!< "0xFF"
    PKT2_HEADER2        = 2,            //!< "0xFD"
    PKT2_RESERVED       = 3,            //!< "0x00"
    PKT2_ID             = 4,            //!< ID of Dynamixel device which will receive the Instruction Packet. Range is [0;252] and 254.
    PKT2_LENGTH_L       = 5,            //!< Length of the packet after this field ([error] + number of parameters + 3).
    PKT2_LENGTH_H       = 6,
    PKT2_INSTRUCTION    = 7,
    PKT2_ERROR          = 8,            //!< Instruction Packet's error. Only used with status packet.
    PKT2_PARAMETER      = 8,            //!< Note: parameter field as an address of 9 (and not 8) when found in status packet.
};

/* ************************************************************************** */

unsigned short crc_table[256] =
{
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

/* ************************************************************************** */

Dynamixel::Dynamixel()
{
    //
}

Dynamixel::~Dynamixel()
{
    serialTerminate();
}

int Dynamixel::serialInitialize(std::string &devicePath, const int baud)
{
    int status = 0;

    if (serial != nullptr)
    {
        serialTerminate();
    }

    // Instanciate a different serial subclass, depending on the current OS
#if defined(FEATURE_QTSERIAL)
    //serial = new SerialPortQt(devicePath, baud, serialDevice, servoSerie);
#else
#if defined(__linux__) || defined(__gnu_linux)
    serial = new SerialPortLinux(devicePath, baud, serialDevice, servoSerie);
#elif defined(_WIN32) || defined(_WIN64)
    serial = new SerialPortWindows(devicePath, baud, serialDevice, servoSerie);
#elif defined(__APPLE__) || defined(__MACH__)
    serial = new SerialPortMacOS(devicePath, baud, serialDevice, servoSerie);
#else
    #error "No compatible operating system detected!"
#endif
#endif

    // Initialize the serial link
    if (serial != nullptr)
    {
        status = serial->openLink();

        if (status > 0)
        {
            TRACE_INFO(DXL, "> Serial interface successfully opened on '%s' @ %i bps", devicePath.c_str(), baud);
        }
    }
    else
    {
        TRACE_ERROR(DXL, "> Failed to open serial interface on '%s' @ %i bps. Exiting...", devicePath.c_str(), baud);
    }

    return status;
}

void Dynamixel::serialTerminate()
{
    if (serial != nullptr)
    {
        // Close serial link
        serial->closeLink();
        delete serial;
        serial = nullptr;

        // Clear incoming packet?
        rxPacketSize = 0;
        memset(rxPacket, 0, sizeof(rxPacket));
    }
}

std::string Dynamixel::serialGetCurrentDevice()
{
    std::string serialName;

    if (serial != nullptr)
    {
        serialName = serial->getDevicePath();
    }
    else
    {
        serialName = "unknown";
    }

    return serialName;
}

std::vector <std::string> Dynamixel::serialGetAvailableDevices()
{
    std::vector <std::string> devices;

    if (serial == nullptr)
    {
        TRACE_ERROR(DXL, "Serial interface is not initialized!");
    }
    else
    {
        devices = serial->scanSerialPorts();
    }

    return devices;
}

void Dynamixel::serialSetLatency(int latency)
{
    serial->setLatency(latency);
}

void Dynamixel::setAckPolicy(int ack)
{
    if (ackPolicy >= ACK_NO_REPLY && ack <= ACK_REPLY_ALL)
    {
        ackPolicy = ack;
    }
    else
    {
        TRACE_ERROR(DXL, "Invalid ack policy: '%i', not in [0;2] range.", ack);
    }
}

void Dynamixel::dxl_tx_packet()
{
    if (serial == nullptr)
    {
        TRACE_ERROR(DXL, "Serial interface is not initialized!");
        return;
    }

    if (commLock == 1)
    {
        return;
    }
    commLock = 1;

    // Make sure serial link is "clean"
    if (commStatus == COMM_RXTIMEOUT || commStatus == COMM_RXCORRUPT)
    {
        serial->flush();
    }

    // Make sure the packet is properly formed
    if (dxl_validate_packet() == 0)
    {
        return;
    }

    // Generate a checksum and write in into the packet
    dxl_checksum_packet();

    // Send packet
    unsigned char txPacketSize = dxl_get_txpacket_size();
    unsigned char txPacketSizeSent = 0;

    if (serial != nullptr)
    {
        txPacketSizeSent = serial->tx(txPacket, txPacketSize);
    }
    else
    {
        TRACE_ERROR(DXL, "Serial interface has been destroyed!");
        return;
    }

    // Check if we send the whole packet
    if (txPacketSize != txPacketSizeSent)
    {
        commStatus = COMM_TXFAIL;
        commLock = 0;
        return;
    }

    // Set a timeout for the response packet
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        // 11 is the min size of a v2 status packet
        if (txPacket[PKT1_INSTRUCTION] == INST_READ)
        {
            serial->setTimeOut(11 + make_short_word(txPacket[PKT2_PARAMETER+2], txPacket[PKT2_PARAMETER+3]));
        }
        else
        {
            serial->setTimeOut(11);
        }
    }
    else
    {
        // 6 is the min size of a v1 status packet
        if (txPacket[PKT1_INSTRUCTION] == INST_READ)
        {
            serial->setTimeOut(6 + txPacket[PKT1_PARAMETER+1]);
        }
        else
        {
            serial->setTimeOut(6);
        }
    }

    commStatus = COMM_TXSUCCESS;
}

void Dynamixel::dxl_rx_packet()
{
    if (serial == nullptr)
    {
        TRACE_ERROR(DXL, "Serial interface is not initialized!");
        return;
    }

    // No lock mean no packet has just been sent, so why wait for an answer (?)
    if (commLock == 0)
    {
        return;
    }

    // Packet sent to a broadcast address? No need to wait for a status packet.
    if ((protocolVersion == PROTOCOL_DXLv1 && txPacket[PKT1_ID] == BROADCAST_ID) ||
        (protocolVersion == PROTOCOL_DXLv2 && txPacket[PKT2_ID] == BROADCAST_ID))
    {
        commStatus = COMM_RXSUCCESS;
        commLock = 0;
        return;
    }

    // Minimum status packet size estimation
    if (commStatus == COMM_TXSUCCESS)
    {
        // Min size with protocol v2 is 11, for v1 is 6
        (protocolVersion == PROTOCOL_DXLv2) ? rxPacketSize = 11 : rxPacketSize = 6;
        rxPacketSizeReceived = 0;
    }

    int nRead = 0;
    if (serial != nullptr)
    {
        // Receive packet
        nRead = serial->rx((unsigned char*)&rxPacket[rxPacketSizeReceived], rxPacketSize - rxPacketSizeReceived);
        rxPacketSizeReceived += nRead;

        // Check if we received the whole packet
        if (rxPacketSizeReceived < rxPacketSize)
        {
            if (serial->checkTimeOut() == 1)
            {
                if (rxPacketSizeReceived == 0)
                {
                    commStatus = COMM_RXTIMEOUT;
                }
                else
                {
                    commStatus = COMM_RXCORRUPT;
                }

                commLock = 0;
                return;
            }
        }
    }
    else
    {
        TRACE_ERROR(DXL, "Serial interface has been destroyed!");
        return;
    }

    // Find packet header
    unsigned char i, j;
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        for (i = 0; i < (rxPacketSizeReceived - 1); i++)
        {
            if (rxPacket[i] == 0xFF &&
                rxPacket[i+1] == 0xFF &&
                rxPacket[i+2] == 0xFD &&
                rxPacket[i+3] == 0x00)
            {
                break;
            }
            else if (i == (rxPacketSizeReceived - 2) &&
                     rxPacket[rxPacketSizeReceived-1] == 0xFF)
            {
                break;
            }
        }

        if (i > 0)
        {
            for (j = 0; j < (rxPacketSizeReceived-i); j++)
            {
                rxPacket[j] = rxPacket[j + i];
            }

            rxPacketSizeReceived -= i;
        }
    }
    else
    {
        for (i = 0; i < (rxPacketSizeReceived - 1); i++)
        {
            if (rxPacket[i] == 0xFF &&
                rxPacket[i+1] == 0xFF)
            {
                break;
            }
            else if (i == (rxPacketSizeReceived - 2) &&
                     rxPacket[rxPacketSizeReceived-1] == 0xFF)
            {
                break;
            }
        }

        if (i > 0)
        {
            for (j = 0; j < (rxPacketSizeReceived-i); j++)
            {
                rxPacket[j] = rxPacket[j + i];
            }

            rxPacketSizeReceived -= i;
        }
    }

    // Incomplete packet?
    if (rxPacketSizeReceived < rxPacketSize)
    {
        commStatus = COMM_RXWAITING;
        return;
    }

    // Check ID pairing
    if (((protocolVersion == PROTOCOL_DXLv1) && (txPacket[PKT1_ID] != rxPacket[PKT1_ID])) ||
        ((protocolVersion == PROTOCOL_DXLv2) && (txPacket[PKT2_ID] != rxPacket[PKT2_ID])))
    {
        commStatus = COMM_RXCORRUPT;
        commLock = 0;
        return;
    }

    // Rx packet size
    rxPacketSize = dxl_get_rxpacket_size();

    if (rxPacketSizeReceived < rxPacketSize)
    {
        nRead = serial->rx(&rxPacket[rxPacketSizeReceived], rxPacketSize - rxPacketSizeReceived);
        rxPacketSizeReceived += nRead;

        if (rxPacketSizeReceived < rxPacketSize)
        {
            commStatus = COMM_RXWAITING;
            return;
        }
    }

    // Generate a checksum of the incoming packet
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        unsigned short crc = dxl2_checksum_packet(rxPacket, rxPacketSize);

        // Compare it with the internal packet checksum
        if (rxPacket[rxPacketSize - 2] != get_lowbyte(crc) &&
            rxPacket[rxPacketSize - 1] != get_highbyte(crc))
        {
            commStatus = COMM_RXCORRUPT;
            commLock = 0;
            return;
        }
    }
    else
    {
        unsigned char checksum = dxl1_checksum_packet(rxPacket, dxl_get_rxpacket_length_field());

        // Compare it with the internal packet checksum
        if (rxPacket[rxPacketSize - 1] != checksum)
        {
            commStatus = COMM_RXCORRUPT;
            commLock = 0;
            return;
        }
    }

    commStatus = COMM_RXSUCCESS;
    commLock = 0;
}

void Dynamixel::dxl_txrx_packet(int ack)
{
#ifdef LATENCY_TIMER
    // Latency timer for a complete transaction (instruction sent and status received)
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    start = std::chrono::high_resolution_clock::now();
#endif

    dxl_tx_packet();

    if (commStatus != COMM_TXSUCCESS)
    {
        TRACE_ERROR(DXL, "Unable to send TX packet on serial link: '%s'", serialGetCurrentDevice().c_str());
        return;
    }

    // Depending on 'ackPolicy' value and current instruction, we wait for an answer to the packet we just sent
    if (ack == ACK_DEFAULT)
    {
        ack = ackPolicy;
    }

    if (ack != ACK_NO_REPLY)
    {
        int cmd = 0;
        if (protocolVersion == PROTOCOL_DXLv2)
        {
            cmd = txPacket[PKT2_INSTRUCTION];
        }
        else
        {
            cmd = txPacket[PKT1_INSTRUCTION];
        }

        if ((ack == ACK_REPLY_ALL) ||
            (ack == ACK_REPLY_READ && cmd == INST_READ))
        {
            do {
                dxl_rx_packet();
            }
            while (commStatus == COMM_RXWAITING);
        }
        else
        {
            commStatus = COMM_RXSUCCESS;
            commLock = 0;
        }
    }
    else
    {
        commStatus = COMM_RXSUCCESS;
        commLock = 0;
    }

#ifdef PACKET_DEBUGGER
    printTxPacket();
    printRxPacket();
#endif

#ifdef LATENCY_TIMER
    end = std::chrono::high_resolution_clock::now();
    int loopd = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    TRACE_1(DXL, "TX > RX loop: %iµs", loopd);
#endif
}

// Low level API
////////////////////////////////////////////////////////////////////////////////

void Dynamixel::dxl_set_txpacket_header()
{
    txPacket[PKT1_HEADER0] = 0xFF;
    txPacket[PKT1_HEADER1] = 0xFF;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_HEADER2] = 0xFD;
        txPacket[PKT2_RESERVED] = 0x00;
    }
}

void Dynamixel::dxl_set_txpacket_id(int id)
{
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
    }
}

void Dynamixel::dxl_set_txpacket_length_field(int length)
{
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_LENGTH_L] = get_lowbyte(length);
        txPacket[PKT2_LENGTH_H] = get_highbyte(length);
    }
    else
    {
        txPacket[PKT1_LENGTH] = get_lowbyte(length);
    }
}

void Dynamixel::dxl_set_txpacket_instruction(int instruction)
{
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_INSTRUCTION] = get_lowbyte(instruction);
    }
    else
    {
        txPacket[PKT1_INSTRUCTION] = get_lowbyte(instruction);
    }
}

void Dynamixel::dxl_set_txpacket_parameter(int index, int value)
{
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_PARAMETER+index] = get_lowbyte(value);
    }
    else
    {
        txPacket[PKT1_PARAMETER+index] = get_lowbyte(value);
    }
}

void Dynamixel::dxl_checksum_packet()
{
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        // Generate checksum
        int packetSize = dxl_get_txpacket_size();
        unsigned short crc = dxl2_checksum_packet(txPacket, packetSize);

        // Write checksum into the last two bytes of the packet
        txPacket[packetSize - 2] = get_lowbyte(crc);
        txPacket[packetSize - 1] = get_highbyte(crc);
    }
    else
    {
        // Generate checksum
        unsigned char checksum = dxl1_checksum_packet(txPacket, dxl_get_txpacket_length_field());

        // Write checksum into the last byte of the packet
        txPacket[dxl_get_txpacket_size() - 1] = checksum;
    }
}

unsigned char Dynamixel::dxl1_checksum_packet(unsigned char *packetData, const int packetLengthField)
{
    unsigned char checksum = 0;

    for (unsigned char i = 0; i < (packetLengthField + 1); i++) // 'length field + 1': whut
    {
        checksum += packetData[i+2];
    }
    checksum = ~checksum;

    return checksum;
}

unsigned short Dynamixel::dxl2_checksum_packet(unsigned char *packetData, const int packetSize)
{
    unsigned short crc = 0;
    unsigned short i = 0, j = 0;

    for(j = 0; j < (packetSize - 2); j++) // 'size - 2': do not CRC16 the CRC fields!
    {
        i = ((crc >> 8) ^ *packetData++) & 0xFF;
        crc = (crc << 8) ^ crc_table[i];
    }

    return crc;
}

int Dynamixel::dxl_get_txpacket_length_field()
{
    int size = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        size = make_short_word(txPacket[PKT2_LENGTH_L], txPacket[PKT2_LENGTH_H]);
    }
    else
    {
        size = static_cast<int>(txPacket[PKT1_LENGTH]);
    }

    return size;
}

int Dynamixel::dxl_get_txpacket_size()
{
    int size = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        // There is 7 bytes before the length field
        size = dxl_get_txpacket_length_field() + 7;
    }
    else
    {
        // There is 4 bytes before the length field
        size = dxl_get_txpacket_length_field() + 4;
    }

    return size;
}

int Dynamixel::dxl_validate_packet()
{
    int retcode = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        retcode = dxl2_validate_packet();
    }
    else
    {
        retcode = dxl1_validate_packet();
    }

    return retcode;
}

int Dynamixel::dxl1_validate_packet()
{
    int retcode = 1;

    // Check if packet size is valid
    if (dxl_get_txpacket_size() > MAX_PACKET_LENGTH_dxlv1)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Check if packet instruction is valid
    if (txPacket[PKT1_INSTRUCTION] != INST_PING &&
        txPacket[PKT1_INSTRUCTION] != INST_READ &&
        txPacket[PKT1_INSTRUCTION] != INST_WRITE &&
        txPacket[PKT1_INSTRUCTION] != INST_REG_WRITE &&
        txPacket[PKT1_INSTRUCTION] != INST_ACTION &&
        txPacket[PKT1_INSTRUCTION] != INST_SYNC_READ)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Write sync header
    dxl_set_txpacket_header();

    return retcode;
}

int Dynamixel::dxl2_validate_packet()
{
    int retcode = 1;

    // Check if packet size is valid
    if (dxl_get_txpacket_size() > MAX_PACKET_LENGTH_dxlv2)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Check if packet instruction is valid
    if (txPacket[PKT2_INSTRUCTION] != INST_PING &&
        txPacket[PKT2_INSTRUCTION] != INST_READ &&
        txPacket[PKT2_INSTRUCTION] != INST_WRITE &&
        txPacket[PKT2_INSTRUCTION] != INST_REG_WRITE &&
        txPacket[PKT2_INSTRUCTION] != INST_ACTION &&
        txPacket[PKT2_INSTRUCTION] != INST_FACTORY_RESET &&
        txPacket[PKT2_INSTRUCTION] != INST_REBOOT &&
        txPacket[PKT2_INSTRUCTION] != INST_STATUS &&
        txPacket[PKT2_INSTRUCTION] != INST_SYNC_READ &&
        txPacket[PKT2_INSTRUCTION] != INST_SYNC_WRITE &&
        txPacket[PKT2_INSTRUCTION] != INST_BULK_READ &&
        txPacket[PKT2_INSTRUCTION] != INST_BULK_WRITE)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Write sync header
    dxl_set_txpacket_header();

    return retcode;
}

int Dynamixel::dxl_get_rxpacket_error()
{
    int status = 0;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        status = (rxPacket[PKT2_ERROR] & 0xFD);
    }
    else
    {
        status = (rxPacket[PKT1_ERRBIT] & 0xFD);
    }

    return status;
}

int Dynamixel::dxl_get_rxpacket_size()
{
    int size = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        // There is 7 bytes before the length field
        size = dxl_get_rxpacket_length_field() + 7;
    }
    else
    {
        // There is 4 bytes before the length field
        size = dxl_get_rxpacket_length_field() + 4;
    }

    return size;
}

int Dynamixel::dxl_get_rxpacket_length_field()
{
    int size = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        size = make_short_word(rxPacket[PKT2_LENGTH_L], rxPacket[PKT2_LENGTH_H]);
    }
    else
    {
        size = static_cast<int>(rxPacket[PKT1_LENGTH]);
    }

    return size;
}

int Dynamixel::dxl_get_rxpacket_parameter(int index)
{
    int value = -1;

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        // +1 represent the error field integrated
        value = static_cast<int>(rxPacket[PKT2_PARAMETER + 1 + index]);
    }
    else
    {
        value = static_cast<int>(rxPacket[PKT1_PARAMETER + index]);
    }

    return value;
}

int Dynamixel::dxl_get_last_packet_id()
{
    int id = 0;

    // We want to use the ID of the last status packet received through the serial link
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        id = (rxPacket[PKT2_ID]);
    }
    else
    {
        id = (rxPacket[PKT1_ID]);
    }

    // In case no status packet has been received (ex: RX timeout) we try to use the ID from the last packet sent
    if (id == 0)
    {
        if (protocolVersion == PROTOCOL_DXLv2)
        {
            id = (txPacket[PKT2_ID]);
        }
        else
        {
            id = (txPacket[PKT1_ID]);
        }
    }

    return id;
}

int Dynamixel::dxl_get_com_status()
{
    return commStatus;
}

int Dynamixel::dxl_get_com_error()
{
    if (commStatus < 0)
    {
        return commStatus;
    }

    return 0;
}

int Dynamixel::dxl_get_com_error_count()
{
    if (commStatus < 0)
    {
        return 1;
    }

    return 0;
}

int Dynamixel::dxl_print_error()
{
    int id = dxl_get_last_packet_id(); // Get device id which produce the error
    int error = 0;

    switch (commStatus)
    {
    case COMM_TXSUCCESS:
    case COMM_RXSUCCESS:
        // Get error bitfield
        error = dxl_get_rxpacket_error();

        if (protocolVersion == PROTOCOL_DXLv2)
        {
            switch (error)
            {
                case ERRBIT2_RESULT:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Failed processing of instruction packet!", id);
                    break;
                case ERRBIT2_INSTRUCTION:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Undefined instruction OR action without 'Reg Write'!", id);
                    break;
                case ERRBIT2_CHECKSUM:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Packet checksum value doesn't compute!", id);
                    break;
                case ERRBIT2_DATA_RANGE:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Value out of range!", id);
                    break;
                case ERRBIT2_DATA_LENGTH:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Data length shorter than required length for the corresponding address!", id);
                    break;
                case ERRBIT2_DATA_LIMIT:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Data length longer than required length for the corresponding address!", id);
                    break;
                case ERRBIT2_ACCESS:
                    TRACE_ERROR(DXL, "[#%i] Protocol Error: Access violation! (Please check for read/write only register; or ROM lock enable)", id);
                    break;
                default:
                    error = 0;
                    break;
            }
        }
        else
        {
            if (error & ERRBIT1_VOLTAGE)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Input voltage error!", id);
            if (error & ERRBIT1_ANGLE_LIMIT)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Angle limit error!", id);
            if (error & ERRBIT1_OVERHEAT)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Overheat error!", id);
            if (error & ERRBIT1_RANGE)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Out of range value error!", id);
            if (error & ERRBIT1_CHECKSUM)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Checksum error!", id);
            if (error & ERRBIT1_OVERLOAD)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Overload error!", id);
            if (error & ERRBIT1_INSTRUCTION)
                TRACE_ERROR(DXL, "[#%i] Protocol Error: Instruction code error!", id);
        }
        break;

    case COMM_UNKNOWN:
        TRACE_ERROR(DXL, "[#%i] COMM_UNKNOWN: Unknown communication error!", id);
        break;

    case COMM_TXFAIL:
        TRACE_ERROR(DXL, "[#%i] COMM_TXFAIL: Failed transmit instruction packet!", id);
        break;

    case COMM_TXERROR:
        TRACE_ERROR(DXL, "[#%i] COMM_TXERROR: Incorrect instruction packet!", id);
        break;

    case COMM_RXFAIL:
        TRACE_ERROR(DXL, "[#%i] COMM_RXFAIL: Failed get status packet from device!", id);
        break;

    case COMM_RXWAITING:
        TRACE_ERROR(DXL, "[#%i] COMM_RXWAITING: Now recieving status packet!", id);
        break;

    case COMM_RXTIMEOUT:
        TRACE_ERROR(DXL, "[#%i] COMM_RXTIMEOUT: Timeout reached while waiting for a status packet!", id);
        break;

    case COMM_RXCORRUPT:
        TRACE_ERROR(DXL, "[#%i] COMM_RXCORRUPT: Status packet is corrupted!", id);
        break;

    default:
        TRACE_ERROR(DXL, "[#%i] commStatus has an unknown error code: '%i'", id, commStatus);
        break;
    }

    return error;
}

void Dynamixel::printRxPacket()
{
    printf("Packet recv [ ");
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        printf("0x%.2X 0x%.2X 0x%.2X 0x%.2X ", rxPacket[0], rxPacket[1], rxPacket[2], rxPacket[3]);
        printf("0x%.2X ", rxPacket[4]);
        printf("{0x%.2X 0x%.2X} ", rxPacket[5], rxPacket[6]);
        printf("(0x%.2X) ", rxPacket[7]);

        int i = 0;
        for (i = 0; i < (rxPacketSize - 2); i++)
        {
            printf("0x%.2X ", rxPacket[i]);
        }
        printf("{0x%.2X 0x%.2X} ", rxPacket[rxPacketSize-2], rxPacket[rxPacketSize-1]);
    }
    else
    {
        printf("0x%.2X 0x%.2X ", rxPacket[0], rxPacket[1]);
        printf("0x%.2X ", rxPacket[2]);
        printf("{0x%.2X} ", rxPacket[3]);
        printf("0x%.2X ", rxPacket[4]);

        // Packet header is 5 bytes (counting length field)
        for (int i = 5; i < (rxPacketSize - 1); i++)
        {
            printf("0x%.2X ", rxPacket[i]);
        }
        printf("{0x%.2X} ", rxPacket[rxPacketSize-1]);
    }
    printf("]\n");
}

void Dynamixel::printTxPacket()
{
    int packetSize = dxl_get_txpacket_size();

    printf("Packet sent [ ");
    if (protocolVersion == PROTOCOL_DXLv2)
    {
        printf("0x%.2X 0x%.2X 0x%.2X 0x%.2X ", txPacket[0], txPacket[1], txPacket[2], txPacket[3]);
        printf("0x%.2X ", txPacket[4]);
        printf("{0x%.2X 0x%.2X} ", txPacket[5], txPacket[6]);
        printf("(0x%.2X) ", txPacket[7]);
        for (int i = 0; i < (packetSize - 2); i++)
        {
            printf("0x%.2X ", txPacket[i]);
        }
        printf("{0x%.2X 0x%.2X} ", txPacket[packetSize-2], txPacket[packetSize-1]);
    }
    else
    {
        printf("0x%.2X 0x%.2X ", txPacket[0], txPacket[1]);
        printf("0x%.2X ", txPacket[2]);
        printf("{0x%.2X} ", txPacket[3]);
        printf("(0x%.2X) ", txPacket[4]);
        for (int i = 5; i < (packetSize - 1); i++)
        {
            printf("0x%.2X ", txPacket[i]);
        }
        printf("{0x%.2X} ", txPacket[packetSize-1]);
    }
    printf("]\n");
}

bool Dynamixel::dxl_ping(const int id, PingResponse *status, const int ack)
{
    bool retcode = false;

    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_PING;
        txPacket[PKT2_LENGTH_L] = 3;
        txPacket[PKT2_LENGTH_H] = 0;
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
        txPacket[PKT1_INSTRUCTION] = INST_PING;
        txPacket[PKT1_LENGTH] = 2;
    }

    dxl_txrx_packet(ack);

    if (commStatus == COMM_RXSUCCESS)
    {
        retcode = true;

        if (status != nullptr)
        {
            if (protocolVersion == PROTOCOL_DXLv2)
            {
                status->model_number = make_short_word(rxPacket[PKT2_PARAMETER+1], rxPacket[PKT2_PARAMETER+2]);
                status->firmware_version = rxPacket[PKT2_PARAMETER+3];
            }
            else
            {
                // Emulate ping response from protocol v2
                status->model_number = dxl_read_word(id, 0, ack);
                status->firmware_version = dxl_read_byte(id, 2, ack);
            }
        }
    }

    return retcode;
}

void Dynamixel::dxl_reset(const int id, int setting, const int ack)
{
    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        if (setting != RESET_ALL &&
            setting != RESET_ALL_EXCEPT_ID &&
            setting != RESET_ALL_EXCEPT_ID_BAUDRATE)
        {
            setting = 0xFF;
        }

        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_FACTORY_RESET;
        txPacket[PKT2_PARAMETER] = setting;
        txPacket[PKT2_LENGTH_L] = 3;
        txPacket[PKT2_LENGTH_H] = 0;
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
        txPacket[PKT1_INSTRUCTION] = INST_FACTORY_RESET;
        txPacket[PKT1_LENGTH] = 2;
    }

    dxl_txrx_packet(ack);
}

void Dynamixel::dxl_reboot(const int id, const int ack)
{
    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_REBOOT;
        txPacket[PKT2_LENGTH_L] = 3;
        txPacket[PKT2_LENGTH_H] = 0;

        dxl_txrx_packet(ack);
    }
    else
    {
        commStatus = COMM_TXFAIL;
        TRACE_ERROR(DXL, "'Reboot' instruction not available with protocol v1!");
    }
}

void Dynamixel::dxl_action(const int id, const int ack)
{
    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_ACTION;
        txPacket[PKT2_LENGTH_L] = 3;
        txPacket[PKT2_LENGTH_H] = 0;
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
        txPacket[PKT1_INSTRUCTION] = INST_ACTION;
        txPacket[PKT1_LENGTH] = 2;
    }

    dxl_txrx_packet(ack);
}

int Dynamixel::dxl_read_byte(const int id, const int address, const int ack)
{
    int value = -1;

    if (id == 254)
    {
        TRACE_ERROR(DXL, "Cannot send 'Read' instruction to broadcast address!");
    }
    else if (ack == ACK_NO_REPLY)
    {
        TRACE_ERROR(DXL, "Cannot send 'Read' instruction if ACK_NO_REPLY is set!");
    }
    else
    {
        while(commLock);

        if (protocolVersion == PROTOCOL_DXLv2)
        {
            txPacket[PKT2_ID] = get_lowbyte(id);
            txPacket[PKT2_INSTRUCTION] = INST_READ;
            txPacket[PKT2_PARAMETER] = get_lowbyte(address);
            txPacket[PKT2_PARAMETER+1] = get_highbyte(address);
            txPacket[PKT2_PARAMETER+2] = 1;
            txPacket[PKT2_PARAMETER+3] = 0;
            txPacket[PKT2_LENGTH_L] = 7;
            txPacket[PKT2_LENGTH_H] = 0;
        }
        else
        {
            txPacket[PKT1_ID] = get_lowbyte(id);
            txPacket[PKT1_INSTRUCTION] = INST_READ;
            txPacket[PKT1_PARAMETER] = get_lowbyte(address);
            txPacket[PKT1_PARAMETER+1] = 1;
            txPacket[PKT1_LENGTH] = 4;
        }

        dxl_txrx_packet(ack);

        if ((ack == ACK_DEFAULT && ackPolicy > ACK_NO_REPLY) ||
            (ack > ACK_NO_REPLY))
        {
            if (commStatus == COMM_RXSUCCESS)
            {
                if (protocolVersion == PROTOCOL_DXLv2)
                {
                    value = static_cast<int>(rxPacket[PKT2_PARAMETER+1]);
                }
                else
                {
                    value = static_cast<int>(rxPacket[PKT1_PARAMETER]);
                }
            }
            else
            {
                value = commStatus;
            }
        }
    }

    return value;
}

void Dynamixel::dxl_write_byte(const int id, const int address, const int value, const int ack)
{
    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_WRITE;
        txPacket[PKT2_PARAMETER] = get_lowbyte(address);
        txPacket[PKT2_PARAMETER+1] = get_highbyte(address);
        txPacket[PKT2_PARAMETER+2] = get_lowbyte(value);
        txPacket[PKT2_LENGTH_L] = 6;
        txPacket[PKT2_LENGTH_H] = 0;
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
        txPacket[PKT1_INSTRUCTION] = INST_WRITE;
        txPacket[PKT1_PARAMETER] = get_lowbyte(address);
        txPacket[PKT1_PARAMETER+1] = get_lowbyte(value);
        txPacket[PKT1_LENGTH] = 4;
    }

    dxl_txrx_packet(ack);
}

int Dynamixel::dxl_read_word(const int id, const int address, const int ack)
{
    int value = -1;

    if (id == 254)
    {
        TRACE_ERROR(DXL, "Error! Cannot send 'Read' instruction to broadcast address!");
    }
    else if (ack == ACK_NO_REPLY)
    {
        TRACE_ERROR(DXL, "Error! Cannot send 'Read' instruction if ACK_NO_REPLY is set!");
    }
    else
    {
        while(commLock);

        if (protocolVersion == PROTOCOL_DXLv2)
        {
            txPacket[PKT2_ID] = get_lowbyte(id);
            txPacket[PKT2_INSTRUCTION] = INST_READ;
            txPacket[PKT2_PARAMETER] = get_lowbyte(address);
            txPacket[PKT2_PARAMETER+1] = get_highbyte(address);
            txPacket[PKT2_PARAMETER+2] = 2;
            txPacket[PKT2_PARAMETER+3] = 0;
            txPacket[PKT2_LENGTH_L] = 7;
            txPacket[PKT2_LENGTH_H] = 0;
        }
        else
        {
            txPacket[PKT1_ID] = get_lowbyte(id);
            txPacket[PKT1_INSTRUCTION] = INST_READ;
            txPacket[PKT1_PARAMETER] = get_lowbyte(address);
            txPacket[PKT1_PARAMETER+1] = 2;
            txPacket[PKT1_LENGTH] = 4;
        }

        dxl_txrx_packet(ack);

        if ((ack == ACK_DEFAULT && ackPolicy > ACK_NO_REPLY) ||
            (ack > ACK_NO_REPLY))
        {
            if (commStatus == COMM_RXSUCCESS)
            {
                if (protocolVersion == PROTOCOL_DXLv2)
                {
                    value = make_short_word(rxPacket[PKT2_PARAMETER+1], rxPacket[PKT2_PARAMETER+2]);
                }
                else
                {
                    value = make_short_word(rxPacket[PKT1_PARAMETER], rxPacket[PKT1_PARAMETER+1]);
                }
            }
            else
            {
                value = commStatus;
            }
        }
    }

    return value;
}

void Dynamixel::dxl_write_word(const int id, const int address, const int value, const int ack)
{
    while(commLock);

    if (protocolVersion == PROTOCOL_DXLv2)
    {
        txPacket[PKT2_ID] = get_lowbyte(id);
        txPacket[PKT2_INSTRUCTION] = INST_WRITE;
        txPacket[PKT2_PARAMETER] = get_lowbyte(address);
        txPacket[PKT2_PARAMETER+1] = get_highbyte(address);
        txPacket[PKT2_PARAMETER+2] = get_lowbyte(value);
        txPacket[PKT2_PARAMETER+3] = get_highbyte(value);
        txPacket[PKT2_LENGTH_L] = 7;
        txPacket[PKT2_LENGTH_H] = 0;
    }
    else
    {
        txPacket[PKT1_ID] = get_lowbyte(id);
        txPacket[PKT1_INSTRUCTION] = INST_WRITE;
        txPacket[PKT1_PARAMETER] = get_lowbyte(address);
        txPacket[PKT1_PARAMETER+1] = get_lowbyte(value);
        txPacket[PKT1_PARAMETER+2] = get_highbyte(value);
        txPacket[PKT1_LENGTH] = 5;
    }

    dxl_txrx_packet(ack);
}
