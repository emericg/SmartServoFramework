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
 * \file HerkuleX.h
 * \date 07/07/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#ifndef HERKULEX_H
#define HERKULEX_H

#include "SerialPortLinux.h"
#include "SerialPortWindows.h"
#include "SerialPortMacOS.h"

#include "Utils.h"
#include "ControlTables.h"
#include "HerkuleXTools.h"

#include <string>
#include <vector>

/*!
 * \brief The HerkuleX class
 *
 * This class provide the low level API to handle servos and "manually" generate
 * instruction packets and send them over a serial link. This API will be used
 * by both "Simple" and "Controller" APIs.
 */
class HerkuleX
{
private:
    SerialPort *serial;                         //!< The serial port instance we are going to use.

    unsigned char txPacket[MAX_PACKET_LENGTH_hkx];//!< TX "instruction" packet buffer
    unsigned char rxPacket[MAX_PACKET_LENGTH_hkx];//!< RX "status" packet buffer
    int rxPacketSize;                           //!< Size of the incoming packet
    int rxPacketSizeReceived;                   //!< Byte(s) of the incoming packet received from the serial link

    /*!
     * The software lock used to lock the serial interface, to avoid concurent
     * reads/writes that would lead to multiplexing and packet corruptions.
     * We need one lock per HerkuleX (or HerkuleXController) instance because
     * we want to keep the ability to use multiple serial interface simultaneously
     * (ex: /dev/tty0 and /dev/ttyUSB0).
     */
    int commLock;
    int commStatus;                             //!< Last communication status

    // Serial communication methods, using one of the SerialPort[Linux/Mac/Windows] implementations.
    void hkx_tx_packet();
    void hkx_rx_packet();
    void hkx_txrx_packet(int ack);

protected:
    HerkuleX();
    virtual ~HerkuleX() = 0;

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
     * \param baud: The baudrate or HerkuleX 'baudnum'.
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
    void hkx_set_txpacket_header();
    void hkx_set_txpacket_id(int id);
    void hkx_set_txpacket_length_field(int length);
    void hkx_set_txpacket_instruction(int instruction);
    void hkx_set_txpacket_parameter(int index, int value);

    //!< Generate a checksum of a packet payload
    unsigned short hkx_checksum_packet(unsigned char *packetData, const int packetSize);

    // TX packet analysis
    int hkx_get_txpacket_size();
    int hkx_get_txpacket_length_field();
    int hkx_validate_packet();

    // RX packet analysis
    int hkx_get_rxpacket_error();
    int hkx_get_rxpacket_status_detail();
    int hkx_get_rxpacket_size();
    int hkx_get_rxpacket_length_field();
    int hkx_get_rxpacket_parameter(int index);

    // Debug methods
    int hkx_get_last_packet_id();
    int hkx_get_com_status();       //!< Get communication status (gbCommStatus) of the latest TX/RX instruction
    int hkx_get_com_error();        //!< Get communication error (if gbCommStatus is not success) of the latest TX/RX instruction
    int hkx_print_error();          //!< Print the last communication error
    int hkx_print_status();         //!< Print the current status
    void printRxPacket();           //!< Print the RX buffer (last packet received)
    void printTxPacket();           //!< Print the TX buffer (last packet sent)

    // Instructions
    bool hkx_ping(const int id, PingResponse *status = NULL, const int ack = ACK_DEFAULT);
    void hkx_reset(const int id, int setting = RESET_ALL_EXCEPT_ID, const int ack = ACK_DEFAULT);
    void hkx_reboot(const int id, const int ack = ACK_DEFAULT);

    // DOCME // Read/write register instructions
    int hkx_read_byte(const int id, const int address, const int register_type, const int ack = ACK_DEFAULT);
    void hkx_write_byte(const int id, const int address, const int value, const int register_type, const int ack = ACK_DEFAULT);
    int hkx_read_word(const int id, const int address, const int register_type, const int ack = ACK_DEFAULT);
    void hkx_write_word(const int id, const int address, const int value, const int register_type, const int ack = ACK_DEFAULT);
    void hkx_i_jog(const int id, const int mode, const int value, const int ack = ACK_DEFAULT);
    void hkx_s_jog(const int id, const int mode, const int value, const int ack = ACK_DEFAULT);

public:
    /*!
     * \brief Get the name of the serial device associated with this HerkuleX instance.
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
     * \brief serialSetLatency
     * \param latency: Latency value in milliseconds.
     */
    void serialSetLatency(int latency);

    /*!
     * \brief setAckPolicy
     * \param ack: Ack policy value, using '::AckPolicy_e' enum.
     */
    void setAckPolicy(int ack);
};

#endif /* HERKULEX_H */
