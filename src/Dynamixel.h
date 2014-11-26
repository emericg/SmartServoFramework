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
 * \file Dynamixel.h
 * \date 05/03/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "SerialPortLinux.h"
#include "SerialPortWindows.h"
#include "SerialPortMacOS.h"

#include "Utils.h"
#include "ControlTables.h"
#include "DynamixelTools.h"

#include <string>
#include <vector>

/*!
 * \brief The Dynamixel class
 *
 * This class provide the low level API to handle servos and "manually" generate
 * instruction packets and send them over a serial link. This API will be used
 * by both "Simple" and "Controller" APIs.
 */
class Dynamixel
{
private:
    SerialPort *serial;                         //!< The serial port instance we are going to use.

    unsigned char txPacket[MAX_PACKET_LENGTH_dxlv1]; //!< TX "instruction" packet buffer
    unsigned char rxPacket[MAX_PACKET_LENGTH_dxlv1]; //!< RX "status" packet buffer
    int rxPacketSize;                           //!< Size of the incoming packet
    int rxPacketSizeReceived;                   //!< Byte(s) of the incoming packet received from the serial link

    /*!
     * The software lock used to lock the serial interface, to avoid concurent
     * reads/writes that would lead to multiplexing and packet corruptions.
     * We need one lock per Dynamixel (or DynamixelController) instance because
     * we want to keep the ability to use multiple serial interface simultaneously
     * (ex: /dev/tty0 and /dev/ttyUSB0).
     */
    int commLock;
    int commStatus;                             //!< Last communication status

    // Serial communication methods, using one of the SerialPort[Linux/Mac/Windows] implementations.
    void dxl_tx_packet();
    void dxl_rx_packet();
    void dxl_txrx_packet(int ack);

protected:
    Dynamixel();
    virtual ~Dynamixel() = 0;

    int serialDevice;                           //!< Serial device in use (if known) using '::SerialDevices_e' enum. Can affect link speed and latency.
    int servoSerie;                             //!< Servo serie using '::ServoDevices_e' enum. Used internally to setup some parameters like maxID, ackPolicy and protocolVersion.

    int protocolVersion;                        //!< Version of the communication protocol in use.
    int maxId;                                  //!< Store in the maximum value for servo IDs.
    int ackPolicy;                              //!< Set the status/ack packet return policy using '::AckPolicy_e' (0: No return; 1: Return for READ commands; 2: Return for all commands).

    // Handle serial link
    ////////////////////////////////////////////////////////////////////////////

    /*!
     * \brief Open a serial link with the given parameters.
     * \param devicePath: The path to the serial device node.
     * \param baud: The baudrate or Dynamixel 'baudnum'.
     * \return 1 if success, 0 otherwise.
     */
    int serialInitialize(std::string &devicePath, const int baud);

    /*!
     * \brief Make sure the serial link is properly closed.
     */
    void serialTerminate();

    // Low level API
    ////////////////////////////////////////////////////////////////////////////

    // TX packet building
    void dxl_set_txpacket_header();
    void dxl_set_txpacket_id(int id);
    void dxl_set_txpacket_length_field(int length);
    void dxl_set_txpacket_instruction(int instruction);
    void dxl_set_txpacket_parameter(int index, int value);

    void dxl_checksum_packet();    //!< Generate and write a checksum of tx packet payload
    unsigned char dxl1_checksum_packet(unsigned char *packetData, const int packetLengthField);
    unsigned short dxl2_checksum_packet(unsigned char *packetData, const int packetSize);

    // TX packet analysis
    int dxl_get_txpacket_size();
    int dxl_get_txpacket_length_field();
    int dxl_validate_packet();
    int dxl1_validate_packet();
    int dxl2_validate_packet();

    // RX packet analysis
    int dxl_get_rxpacket_error();
    int dxl_get_rxpacket_size();
    int dxl_get_rxpacket_length_field();
    int dxl_get_rxpacket_parameter(int index);

    // Debug methods
    int dxl_get_last_packet_id();
    int dxl_get_com_status();       //!< Get communication status (gbCommStatus) of the latest TX/RX instruction
    int dxl_get_com_error();        //!< Get communication error (if gbCommStatus is not success) of the latest TX/RX instruction
    int dxl_print_error();          //!< Print the last communication error
    void printRxPacket();           //!< Print the RX buffer (last packet received)
    void printTxPacket();           //!< Print the TX buffer (last packet sent)

    // Instructions
    bool dxl_ping(const int id, PingResponse *status = NULL, const int ack = ACK_DEFAULT);

    /*!
     * \brief Reset servo control table.
     * \param id: The servo to reset to factory default settings.
     * \param setting: If protocol v2 is used, you can control what to erase using the 'ResetOptions' enum.
     * \param ack: Ack policy in effect.
     *
     * \todo emulate "RESET_ALL_EXCEPT_ID" and "RESET_ALL_EXCEPT_ID_BAUDRATE" settings when using protocol v1?
     * Please note that when using protocol v1, the servo ID will be changed to 1.
     */
    void dxl_reset(const int id, int setting, const int ack = ACK_DEFAULT);
    void dxl_reboot(const int id, const int ack = ACK_DEFAULT);
    void dxl_action(const int id, const int ack = ACK_DEFAULT);

    // DOCME // Read/write register instructions
    int dxl_read_byte(const int id, const int address, const int ack = ACK_DEFAULT);
    void dxl_write_byte(const int id, const int address, const int value, const int ack = ACK_DEFAULT);
    int dxl_read_word(const int id, const int address, const int ack = ACK_DEFAULT);
    void dxl_write_word(const int id, const int address, const int value, const int ack = ACK_DEFAULT);
/*
    // TODO // Reg write
    void dxl_reg_write(const int id, ???)

    // TODO // Sync read/write register instructions
    std::vector <int> dxl_sync_read_byte(std::vector <int> ids, int address);
    void dxl_sync_write_byte(std::vector <int> ids, int address, int value);
    std::vector <int> dxl_sync_read_word(std::vector <int> ids, int address);
    void dxl_sync_write_word(std::vector <int> ids, int address, int value);

    // TODO // Bulk read/write register instructions
    std::vector <int> dxl_bulk_read_byte(std::vector <int> ids, int address);
    void dxl_bulk_write_byte(std::vector <int> ids, int address, int value);
    std::vector <int> dxl_bulk_read_word(std::vector <int> ids, int address);
    void dxl_bulk_write_word(std::vector <int> ids, int address, int value);
*/
public:
    /*!
     * \brief Get the name of the serial device associated with this Dynamixel instance.
     * \return The path to the serial device node (ex: "/dev/ttyUSB0").
     */
    std::string serialGetCurrentDevice();

    /*!
     * \brief Get the available serial devices.
     * \return A list of path to all the serial device nodes available (ex: "/dev/ttyUSB0").
     */
    std::vector <std::string> serialGetAvailableDevices();

    /*!
     * \brief Lock serial interface using a temporary file.
     */
    void serialLockInterface();

    /*!
     * \brief setLatency
     * \param latency: Latency value in milliseconds.
     */
    void serialSetLatency(int latency);

    /*!
     * \brief setAckPolicy
     * \param ack: Ack policy value, using '::AckPolicy_e' enum.
     */
    void setAckPolicy(int ack);
};

#endif /* DYNAMIXEL_H */
