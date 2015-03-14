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
 * \file HerkuleX.cpp
 * \date 07/07/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "HerkuleX.h"

// C++ standard libraries
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>

/* ************************************************************************** */

// Enable latency timer
//#define LATENCY_TIMER

// Enable packet debugger
//#define PACKET_DEBUGGER

/* ************************************************************************** */

/*!
 * \brief The different instructions available with the HerkuleX protocol.
 */
enum HerkuleXProtocol {
    CMD_EEP_WRITE  = 1,
    CMD_EEP_READ   = 2,
    CMD_RAM_WRITE  = 3,
    CMD_RAM_READ   = 4,
    CMD_I_JOG      = 5,
    CMD_S_JOG      = 6,
    CMD_STAT       = 7,
    CMD_ROLLBACK   = 8,
    CMD_REBOOT     = 9,

    ACK_EEP_WRITE  = 0x41,
    ACK_EEP_READ   = 0x42,
    ACK_RAM_WRITE  = 0x43,
    ACK_RAM_READ   = 0x44,
    ACK_I_JOG      = 0x45,
    ACK_S_JOG      = 0x46,
    ACK_STAT       = 0x47,
    ACK_ROLLBACK   = 0x48,
    ACK_REBOOT     = 0x49,
};

/*!
 * \brief Addresses of the various fields forming a packet.
 */
enum {
    PKT_HEADER0     = 0,            //!< "0xFF". The 2 bytes header indicate the beginning of a packet.
    PKT_HEADER1     = 1,            //!< "0xFF"
    PKT_LENGTH      = 2,
    PKT_ID          = 3,            //!< ID of HerkuleX device which will receive the Instruction Packet. Range is [0;254].
    PKT_CMD         = 4,
    PKT_CHECKSUM1   = 5,            //!< Instruction Packet's error. Only used with status packet.
    PKT_CHECKSUM2   = 6,            //!< Note: parameter field as an address of 9 (and not 8) when found in status packet.
    PKT_DATA        = 7,            //!< Note: parameter field as an address of 9 (and not 8) when found in status packet.
};

/* ************************************************************************** */

HerkuleX::HerkuleX():
    serial(NULL),
    rxPacketSize(0),
    rxPacketSizeReceived(0),
    commLock(0),
    commStatus(COMM_RXSUCCESS),
    serialDevice(SERIAL_UNKNOWN),
    servoSerie(SERVO_HERKULEX),
    protocolVersion(1)
{
    memset(txPacket, 0, sizeof(txPacket));
    memset(rxPacket, 0, sizeof(rxPacket));
}

HerkuleX::~HerkuleX()
{
    serialTerminate();
}

int HerkuleX::serialInitialize(std::string &devicePath, const int baud)
{
    int status = 0;

    if (serial != NULL)
    {
        serialTerminate();
    }

    // Instanciate a different serial subclass, depending on the current OS
#if defined(__linux__) || defined(__gnu_linux)
    serial = new SerialPortLinux(devicePath, baud, serialDevice, SERVO_DRS);
#elif defined(_WIN32) || defined(_WIN64)
    serial = new SerialPortWindows(devicePath, baud, serialDevice, SERVO_DRS);
#elif defined(__APPLE__) || defined(__MACH__)
    serial = new SerialPortMacOS(devicePath, baud, serialDevice, SERVO_DRS);
#else
    #error "No compatible operating system detected!"
#endif

    // Initialize the serial link
    if (serial != NULL && serial->openLink() == 1)
    {
        status = 1;
        std::cout << "> Serial interface successfully opened on " << devicePath << " @ " << baud << std::endl;
    }
    else
    {
        std::cerr << "> Failed to open serial interface on " << devicePath << " @ " << baud << ". Exiting..." << std::endl;
    }

    return status;
}

void HerkuleX::serialTerminate()
{
    if (serial != NULL)
    {
        // Close serial link
        serial->closeLink();
        delete serial;
        serial = NULL;

        // Clear incoming packet?
        rxPacketSize = 0;
        memset(rxPacket, 0, sizeof(rxPacket));
    }
}

std::string HerkuleX::serialGetCurrentDevice()
{
    std::string serialName;

    if (serial != NULL)
    {
        serialName = serial->getDevicePath();
    }
    else
    {
        serialName = "unknown";
    }

    return serialName;
}

std::vector <std::string> HerkuleX::serialGetAvailableDevices()
{
    std::vector <std::string> devices;

    if (serial == NULL)
    {
        std::cerr << "Serial interface is not initialized!" << std::endl;
    }
    else
    {
        devices = serial->scanSerialPorts();
    }

    return devices;
}

void HerkuleX::serialLockInterface()
{
    serial->setLock();
}

void HerkuleX::serialSetLatency(int latency)
{
    serial->setLatency(latency);
}

void HerkuleX::setAckPolicy(int ack)
{
    if (ackPolicy >= ACK_NO_REPLY && ack <= ACK_REPLY_ALL)
    {
        ackPolicy = ack;
    }
    else
    {
        std::cerr << "Invalid ack policy: '" << ack << "'', not in [0;2] range." << std::endl;
    }
}

void HerkuleX::hkx_tx_packet()
{
    if (serial == NULL)
    {
        std::cerr << "Serial interface is not initialized!" << std::endl;
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
    if (hkx_validate_packet() == 0)
    {
        return;
    }

    // Generate a checksum and write in into the packet
    int txPacketSize = hkx_get_txpacket_size();
    unsigned short crc = hkx_checksum_packet(txPacket, txPacketSize);

    // Write checksum into the last two bytes of the packet
    txPacket[PKT_CHECKSUM1] = get_lowbyte(crc);
    txPacket[PKT_CHECKSUM2] = get_highbyte(crc);

    // Send packet
    unsigned char txPacketSizeSent = serial->tx(txPacket, txPacketSize);

    // Check if we send the whole packet
    if (txPacketSize != txPacketSizeSent)
    {
        commStatus = COMM_TXFAIL;
        commLock = 0;
        return;
    }

    // Set a timeout for the response packet
    // Min size for an RX packet is 9
    if (txPacket[PKT_CMD] == CMD_EEP_READ || txPacket[PKT_CMD] == CMD_RAM_READ)
    {
        serial->setTimeOut(9 + txPacket[PKT_DATA+1]);
    }
    else
    {
        serial->setTimeOut(9);
    }

    commStatus = COMM_TXSUCCESS;
}

void HerkuleX::hkx_rx_packet()
{
    if (serial == NULL)
    {
        std::cerr << "Serial interface is not initialized!" << std::endl;
        return;
    }

    // No lock mean no packet has just been sent, so why wait for an answer (?)
    if (commLock == 0)
    {
        return;
    }

    // Packet sent to a broadcast address? No need to wait for a status packet.
    if (txPacket[PKT_ID] == BROADCAST_ID)
    {
        commStatus = COMM_RXSUCCESS;
        commLock = 0;
        return;
    }

    // Minimum status packet size estimation
    if (commStatus == COMM_TXSUCCESS)
    {
        // Min size for an RX packet is 9
        rxPacketSize = 9;
        rxPacketSizeReceived = 0;
    }

    // Receive packet
    int nRead = serial->rx((unsigned char*)&rxPacket[rxPacketSizeReceived], rxPacketSize - rxPacketSizeReceived);
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

    // Find packet header
    unsigned char i, j;
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

    // Incomplete packet?
    if (rxPacketSizeReceived < rxPacketSize)
    {
        commStatus = COMM_RXWAITING;
        return;
    }

    // Check ID pairing
    if (txPacket[PKT_ID] != rxPacket[PKT_ID])
    {
        commStatus = COMM_RXCORRUPT;
        commLock = 0;
        return;
    }

    // Rx packet size
    rxPacketSize = hkx_get_rxpacket_size();

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
    {
        unsigned short checksum = hkx_checksum_packet(rxPacket, rxPacketSize);

        // Compare it with the internal packet checksum
        if (rxPacket[PKT_CHECKSUM1] != get_lowbyte(checksum) &&
            rxPacket[PKT_CHECKSUM2] != get_highbyte(checksum))
        {
            commStatus = COMM_RXCORRUPT;
            commLock = 0;
            return;
        }
    }

    commStatus = COMM_RXSUCCESS;
    commLock = 0;
}

void HerkuleX::hkx_txrx_packet(int ack)
{
#ifdef LATENCY_TIMER
    // Latency timer for a complete transaction (instruction sent and status received)
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    start = std::chrono::high_resolution_clock::now();
#endif

    hkx_tx_packet();

    if (commStatus != COMM_TXSUCCESS)
    {
        std::cerr << "Unable to sent TX packet on serial link: " << serialGetCurrentDevice() << "'" << std::endl;
        return;
    }

    // Depending on 'ackPolicy' value and current instruction, we wait for an answer to the packet we just sent
    if (ack == ACK_DEFAULT)
    {
        ack = ackPolicy;
    }

    if (ack != ACK_NO_REPLY)
    {
        int cmd = txPacket[PKT_CMD];

        if ((ack == ACK_REPLY_ALL) ||
            (ack == ACK_REPLY_READ && (cmd == CMD_STAT || cmd == CMD_EEP_READ || cmd == CMD_RAM_READ)))
        {
            do {
                hkx_rx_packet();
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
    std::cout << "TX > RX loop: " << loopd << "µs ("<< 0 << ")" << std::endl;
#endif
}

// Low level API
////////////////////////////////////////////////////////////////////////////////

void HerkuleX::hkx_set_txpacket_header()
{
    txPacket[PKT_HEADER0] = 0xFF;
    txPacket[PKT_HEADER1] = 0xFF;
}

void HerkuleX::hkx_set_txpacket_id(int id)
{
    txPacket[PKT_ID] = get_lowbyte(id);
}

void HerkuleX::hkx_set_txpacket_length_field(int length)
{
    txPacket[PKT_LENGTH] = get_lowbyte(length);
}

void HerkuleX::hkx_set_txpacket_instruction(int instruction)
{
    txPacket[PKT_CMD] = get_lowbyte(instruction);
}

void HerkuleX::hkx_set_txpacket_parameter(int index, int value)
{
        txPacket[PKT_DATA+index] = get_lowbyte(value);
}

unsigned short HerkuleX::hkx_checksum_packet(unsigned char *packetData, const int packetSize)
{
    // Generate checksum
    unsigned short checksum = 0;

    // (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n])&0xFE
    int Sum1 = packetData[PKT_LENGTH];
    Sum1 ^= packetData[PKT_ID];
    Sum1 ^= packetData[PKT_CMD];
    for (int i = PKT_DATA; i < packetSize; i++)
    {
        Sum1 ^= packetData[i];
    }
    Sum1 &= 0xFE;

    //
    int Sum2 = (~Sum1) & 0xFE;

    // Make a word from these two bytes
    checksum  = get_lowbyte(Sum1);
    checksum += get_lowbyte(Sum2) << 8;

    return checksum;
}

int HerkuleX::hkx_get_txpacket_length_field()
{
    // The length field represent the full size of the packet
    return static_cast<int>(txPacket[PKT_LENGTH]);
}

int HerkuleX::hkx_get_txpacket_size()
{
    // The length field represent the full size of the packet
    return static_cast<int>(txPacket[PKT_LENGTH]);
}

int HerkuleX::hkx_validate_packet()
{
    int retcode = 1;

    // Check if packet size is valid
    if (hkx_get_txpacket_size() > MAX_PACKET_LENGTH_hkx)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Check if packet instruction is valid
    if (txPacket[PKT_CMD] != CMD_EEP_WRITE &&
        txPacket[PKT_CMD] != CMD_EEP_READ &&
        txPacket[PKT_CMD] != CMD_RAM_WRITE &&
        txPacket[PKT_CMD] != CMD_RAM_READ &&
        txPacket[PKT_CMD] != CMD_I_JOG &&
        txPacket[PKT_CMD] != CMD_S_JOG &&
        txPacket[PKT_CMD] != CMD_STAT &&
        txPacket[PKT_CMD] != CMD_ROLLBACK &&
        txPacket[PKT_CMD] != CMD_REBOOT)
    {
        commStatus = COMM_TXERROR;
        commLock = 0;
        retcode = 0;
    }

    // Write sync header
    hkx_set_txpacket_header();

    return retcode;
}

int HerkuleX::hkx_get_rxpacket_error()
{
    int error = (rxPacket[rxPacketSize - 2] & 0xFD);
    return error;
}

int HerkuleX::hkx_get_rxpacket_status_detail()
{
    int status = (rxPacket[rxPacketSize - 1] & 0xFD);
    return status;
}

int HerkuleX::hkx_get_rxpacket_size()
{
    // The length field represent the full size of the packet
    return static_cast<int>(rxPacket[PKT_LENGTH]);
}

int HerkuleX::hkx_get_rxpacket_length_field()
{
    // The length field represent the full size of the packet
    return static_cast<int>(rxPacket[PKT_LENGTH]);
}

int HerkuleX::hkx_get_rxpacket_parameter(int index)
{
    return static_cast<int>(rxPacket[PKT_DATA + index]);
}

int HerkuleX::hkx_get_last_packet_id()
{
    // We want to use the ID of the last status packet received through the serial link
    int id = rxPacket[PKT_ID];

    // In case no status packet has been received (ex: RX timeout) we try to use the ID from the last packet sent
    if (id == 0)
    {
        id = txPacket[PKT_ID];
    }

    return id;
}

int HerkuleX::hkx_get_com_status()
{
    return commStatus;
}

int HerkuleX::hkx_get_com_error()
{
    int error = 0;
    if (commStatus > COMM_RXSUCCESS)
    {
        error = commStatus;
    }

    return error;
}

int HerkuleX::hkx_print_error()
{
    int id = hkx_get_last_packet_id(); // Get device id which produce the error
    int error = 0;
    int status = 0;

    switch (commStatus)
    {
    case COMM_RXSUCCESS:
    case COMM_TXSUCCESS:
        // Get error bitfield
        error = hkx_get_rxpacket_error();

        if (error & ERRBIT_VOLTAGE)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_VOLTAGE" << std::endl;
        if (error & ERRBIT_ALLOWED_POT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_ALLOWED_POT" << std::endl;
        if (error & ERRBIT_OVERHEAT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_OVERHEAT" << std::endl;
        if (error & ERRBIT_INVALID_PKT)
        {
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_INVALID_PKT:" << std::endl;

            // Get status bitfield
            status = hkx_get_rxpacket_status_detail();

            if (status & STATBIT_CHECKSUM_FLAG)
                std::cerr << "[#" << id << "]   Packet Error: STATBIT_CHECKSUM_FLAG" << std::endl;
            if (status & STATBIT_UNKWOWN_CMD)
                std::cerr << "[#" << id << "]   Packet Error: STATBIT_UNKWOWN_CMD" << std::endl;
            if (status & STATBIT_RANGE)
                std::cerr << "[#" << id << "]   Packet Error: STATBIT_RANGE" << std::endl;
            if (status & STATBIT_GARBAGE)
                std::cerr << "[#" << id << "]   Packet Error: STATBIT_GARBAGE" << std::endl;
        }
        if (error & ERRBIT_OVERLOAD)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_OVERLOAD" << std::endl;
        if (error & ERRBIT_DRIVER_FAULT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_DRIVER_FAULT" << std::endl;
        if (error & ERRBIT_EEP_REG_DIST)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_EEP_REG_DIST" << std::endl;
        break;

    case COMM_TXFAIL:
        std::cerr << "[#" << id << "] COMM_TXFAIL: Failed transmit instruction packet!" << std::endl;
        break;

    case COMM_TXERROR:
        std::cerr << "[#" << id << "] COMM_TXERROR: Incorrect instruction packet!" << std::endl;
        break;

    case COMM_RXFAIL:
        std::cerr << "[#" << id << "] COMM_RXFAIL: Failed get status packet from device!" << std::endl;
        break;

    case COMM_RXWAITING:
        std::cerr << "[#" << id << "] COMM_RXWAITING: Now recieving status packet!" << std::endl;
        break;

    case COMM_RXTIMEOUT:
        std::cerr << "[#" << id << "] COMM_RXTIMEOUT: Timeout reached while waiting for a status packet!" << std::endl;
        break;

    case COMM_RXCORRUPT:
        std::cerr << "[#" << id << "] COMM_RXCORRUPT: Status packet is corrupted!" << std::endl;
        break;

    default:
        std::cerr << "[#" << id << "] commStatus has an unknown error code: "<< commStatus << std::endl;
        break;
    }

    return error;
}

int HerkuleX::hkx_print_status()
{
    int id = hkx_get_last_packet_id(); // Get device id which produce the error
    int error = 0;
    int status = 0;

    switch (commStatus)
    {
    case COMM_RXSUCCESS:
    case COMM_TXSUCCESS:
        // Get status bitfield
        status = hkx_get_rxpacket_status_detail();

        if (error & ERRBIT_VOLTAGE)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_VOLTAGE" << std::endl;
        if (error & ERRBIT_ALLOWED_POT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_ALLOWED_POT" << std::endl;
        if (error & ERRBIT_OVERHEAT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_OVERHEAT" << std::endl;
        if (error & ERRBIT_INVALID_PKT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_INVALID_PKT" << std::endl;
        if (error & ERRBIT_OVERLOAD)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_OVERLOAD" << std::endl;
        if (error & ERRBIT_DRIVER_FAULT)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_DRIVER_FAULT" << std::endl;
        if (error & ERRBIT_EEP_REG_DIST)
            std::cerr << "[#" << id << "] Protocol Error: ERRBIT_EEP_REG_DIST" << std::endl;

        if (status & STATBIT_MOVING)
            std::cout << "[#" << id << "] Protocol Status: STATBIT_MOVING" << std::endl;
        if (status & STATBIT_INPOSITION)
            std::cout << "[#" << id << "] Protocol Status: STATBIT_INPOSITION" << std::endl;
        if (status & STATBIT_CHECKSUM_FLAG)
            std::cerr << "[#" << id << "] Packet Status: STATBIT_CHECKSUM_FLAG" << std::endl;
        if (status & STATBIT_UNKWOWN_CMD)
            std::cerr << "[#" << id << "] Packet Status: STATBIT_UNKWOWN_CMD" << std::endl;
        if (status & STATBIT_RANGE)
            std::cerr << "[#" << id << "] Packet Status: STATBIT_RANGE" << std::endl;
        if (status & STATBIT_GARBAGE)
            std::cerr << "[#" << id << "] Packet Status: STATBIT_GARBAGE" << std::endl;
        if (status & STATBIT_TORQUE_ON)
            std::cout << "[#" << id << "] Protocol Status: STATBIT_TORQUE_ON" << std::endl;
        break;
    }

    return error;
}

void HerkuleX::printRxPacket()
{
    printf("Packet recv [ ");
    printf("0x%.2X 0x%.2X ", rxPacket[0], rxPacket[1]);
    printf("{0x%.2X} ", rxPacket[2]);
    printf("0x%.2X ", rxPacket[3]);
    printf("(0x%.2X) ", rxPacket[4]);
    printf("{0x%.2X 0x%.2X} ", rxPacket[5], rxPacket[6]);
    for (int i = 7; i < rxPacketSize; i++)
    {
        printf("0x%.2X ", rxPacket[i]);
    }
    printf("]\n");
}

void HerkuleX::printTxPacket()
{
    printf("Packet sent [ ");
    printf("0x%.2X 0x%.2X ", txPacket[0], txPacket[1]);
    printf("{0x%.2X} ", txPacket[2]);
    printf("0x%.2X ", txPacket[3]);
    printf("(0x%.2X) ", txPacket[4]);
    printf("{0x%.2X 0x%.2X} ", txPacket[5], txPacket[6]);
    // The length field represent the full size of the packet
    for (int i = 7; i < txPacket[PKT_LENGTH]; i++)
    {
        printf("0x%.2X ", txPacket[i]);
    }
    printf("]\n");
}

bool HerkuleX::hkx_ping(const int id, PingResponse *status, const int ack)
{
    bool retcode = false;

    while(commLock);

    // We do not use a READ instruction directly instead of STAT, because it may
    // not receive an answer depending on ack policy value.

    txPacket[PKT_ID] = get_lowbyte(id);
    txPacket[PKT_CMD] = CMD_STAT;
    txPacket[PKT_LENGTH] = 7;

    hkx_txrx_packet(ack);

    if (commStatus == COMM_RXSUCCESS)
    {
        retcode = true;

        if (status != NULL)
        {
            // Emulate ping response from Dynamixel protocol v2
            status->model_number = hkx_read_word(id, 0, REGISTER_ROM, ack);
            status->firmware_version = hkx_read_word(id, 2, REGISTER_ROM, ack);
        }
    }

    return retcode;
}

void HerkuleX::hkx_reset(const int id, int setting, const int ack)
{
    while(commLock);

    txPacket[PKT_LENGTH] = 9;
    txPacket[PKT_ID] = get_lowbyte(id);
    txPacket[PKT_CMD] = CMD_ROLLBACK;

    if (setting == RESET_ALL_EXCEPT_ID)
    {
        txPacket[PKT_DATA]   = 1;
        txPacket[PKT_DATA+1] = 0;
    }
    else if (setting == RESET_ALL_EXCEPT_ID_BAUDRATE)
    {
        txPacket[PKT_DATA]   = 1;
        txPacket[PKT_DATA+1] = 1;
    }
    else
    {
        // RESET_ALL
    }

    hkx_txrx_packet(ack);
}

void HerkuleX::hkx_reboot(const int id, const int ack)
{
    while(commLock);

    txPacket[PKT_LENGTH] = 7;
    txPacket[PKT_ID] = get_lowbyte(id);
    txPacket[PKT_CMD] = CMD_REBOOT;

    hkx_txrx_packet(ack);
}

int HerkuleX::hkx_read_byte(const int id, const int address, const int register_type, const int ack)
{
    int value = -1;

    if (id == 254)
    {
        std::cerr << "Error! Cannot send 'Read' instruction to broadcast address!" << std::endl;
    }
    else if (ack == ACK_NO_REPLY)
    {
        std::cerr << "Error! Cannot send 'Read' instruction if ACK_NO_REPLY is set!" << std::endl;
    }
    else
    {
        while(commLock);

        txPacket[PKT_LENGTH] = 7 + 2;
        txPacket[PKT_ID] = get_lowbyte(id);
        if (register_type == REGISTER_RAM)
            txPacket[PKT_CMD] = CMD_RAM_READ;
        else
            txPacket[PKT_CMD] = CMD_EEP_READ;

        txPacket[PKT_DATA] = get_lowbyte(address);
        txPacket[PKT_DATA+1] = 1;
    }

    hkx_txrx_packet(ack);

    if ((ack == ACK_DEFAULT && ackPolicy > ACK_NO_REPLY) ||
        (ack > ACK_NO_REPLY))
    {
        if (commStatus == COMM_RXSUCCESS)
        {
            value = static_cast<int>(rxPacket[PKT_DATA+2]);
        }
    }

    return value;
}

void HerkuleX::hkx_write_byte(const int id, const int address, const int value, const int register_type, const int ack)
{
    while(commLock);

    txPacket[PKT_LENGTH] = 7 + 3;
    txPacket[PKT_ID] = get_lowbyte(id);
    if (register_type == REGISTER_RAM)
        txPacket[PKT_CMD] = CMD_RAM_WRITE;
    else
        txPacket[PKT_CMD] = CMD_EEP_WRITE;

    txPacket[PKT_DATA] = get_lowbyte(address);
    txPacket[PKT_DATA+1] = 1;
    txPacket[PKT_DATA+2] = get_lowbyte(value);

    hkx_txrx_packet(ack);
}

int HerkuleX::hkx_read_word(const int id, const int address, const int register_type, const int ack)
{
    int value = -1;

    if (id == 254)
    {
        std::cerr << "Error! Cannot send 'Read' instruction to broadcast address!" << std::endl;
    }
    else if (ack == ACK_NO_REPLY)
    {
        std::cerr << "Error! Cannot send 'Read' instruction if ACK_NO_REPLY is set!" << std::endl;
    }
    else
    {
        while(commLock);

        txPacket[PKT_LENGTH] = 7 + 2;
        txPacket[PKT_ID] = get_lowbyte(id);
        if (register_type == REGISTER_RAM)
            txPacket[PKT_CMD] = CMD_RAM_READ;
        else
            txPacket[PKT_CMD] = CMD_EEP_READ;

        txPacket[PKT_DATA] = get_lowbyte(address);
        txPacket[PKT_DATA+1] = 2;

        hkx_txrx_packet(ack);

        if ((ack == ACK_DEFAULT && ackPolicy > ACK_NO_REPLY) ||
            (ack > ACK_NO_REPLY))
        {
            if (commStatus == COMM_RXSUCCESS)
            {
                value = make_short_word(rxPacket[PKT_DATA+2], rxPacket[PKT_DATA+3]);
            }
        }
    }

    return value;
}

void HerkuleX::hkx_write_word(const int id, const int address, const int value, const int register_type, const int ack)
{
    while(commLock);

    txPacket[PKT_LENGTH] = 7 + 4;
    txPacket[PKT_ID] = get_lowbyte(id);
    if (register_type == REGISTER_RAM)
        txPacket[PKT_CMD] = CMD_RAM_WRITE;
    else
        txPacket[PKT_CMD] = CMD_EEP_WRITE;

    txPacket[PKT_DATA] = get_lowbyte(address);
    txPacket[PKT_DATA+1] = 2;
    txPacket[PKT_DATA+2] = get_lowbyte(value);
    txPacket[PKT_DATA+3] = get_highbyte(value);

    hkx_txrx_packet(ack);
}

void HerkuleX::hkx_i_jog(const int id, const int mode, const int value, const int ack)
{
    int JOG = 0;
    int SET = 0;
    while(commLock);

    txPacket[PKT_LENGTH] = 7 + 5;
    txPacket[PKT_ID] = get_lowbyte(id);
    txPacket[PKT_CMD] = CMD_I_JOG;

    if (mode == 0) // Position control
    {
        JOG = value; // goal position
        SET = 0x04; // position control with green led
    }
    else // if (mode == 1) // Continuous rotation
    {
        if (value >= 0)
        {
            JOG = value; // speed
        }
        else
        {
            JOG = abs(value); // speed
            JOG += 0x4000; // direction
        }
        SET = 0x0A; // continuous rotation with blue led
    }

    // I_JOG(0)
    txPacket[PKT_DATA]   = get_lowbyte(JOG);
    txPacket[PKT_DATA+1] = get_highbyte(JOG);
    txPacket[PKT_DATA+2] = get_lowbyte(SET);
    txPacket[PKT_DATA+3] = get_lowbyte(id); // id
    txPacket[PKT_DATA+4] = 0x3c; // playtime

    hkx_txrx_packet(ack);
}

void HerkuleX::hkx_s_jog(const int id, const int mode, const int value, const int ack)
{
    int JOG = 0;
    int SET = 0;
    while(commLock);

    txPacket[PKT_LENGTH] = 7 + 5;
    txPacket[PKT_ID] = get_lowbyte(id);
    txPacket[PKT_CMD] = CMD_S_JOG;
    txPacket[PKT_DATA] = 0x3c; // playtime

    if (mode == 0) // Position control
    {
        JOG = value; // goal position
        SET = 0x04; // position control with green led
    }
    else // if (mode == 1) // Continuous rotation
    {
        if (value >= 0)
        {
            JOG = value; // goal position
        }
        else
        {
            JOG = abs(value); // speed
            JOG += 0x4000; // direction
        }
        SET = 0x0A; // continuous rotation with blue led
    }

    // S_JOG(0)
    txPacket[PKT_DATA+1] = get_lowbyte(JOG);
    txPacket[PKT_DATA+2] = get_highbyte(JOG);
    txPacket[PKT_DATA+3] = get_lowbyte(SET);
    txPacket[PKT_DATA+4] = get_lowbyte(id); // id

    hkx_txrx_packet(ack);
}
