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
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef HERKULEX_H
#define HERKULEX_H

#include "SerialPortQt.h"
#include "SerialPortLinux.h"
#include "SerialPortWindows.h"
#include "SerialPortMacOS.h"

#include "Utils.h"
#include "ControlTables.h"
#include "HerkuleXTools.h"

#include <string>
#include <vector>

/*!
 * \brief The HerkuleX communication protocol implementation
 * \todo Rename to HerkuleXProtocol
 *
 * Get more informations on the HerkuleX devices here:
 * http://hovis.co.kr/guide/herkulex_eng.html
 * http://hovis.co.kr/guide/herkulex/drs-0101/[ENG]%20Herkulex%20Manual_20140218.pdf
 * http://hovis.co.kr/guide/herkulex/drs-0401/[Eng]Herkulex0401%20manual_140804.pdf
 * http://hovis.co.kr/guide/herkulex/drs-0601/[Eng]Herkulex0601%20manual_140804.pdf
 *
 * This class provide the low level API to handle communication with servos.
 * It can generate instruction packets and send them over a serial link. This class
 * will be used by both "SimpleAPIs" and "Controllers".
 */
class HerkuleX
{
private:
    SerialPort *serial = nullptr;   //!< The serial port instance we are going to use.

    unsigned char txPacket[MAX_PACKET_LENGTH_hkx] = {0};    //!< TX "instruction" packet buffer
    unsigned char rxPacket[MAX_PACKET_LENGTH_hkx] = {0};    //!< RX "status" packet buffer
    int rxPacketSize = 0;           //!< Size of the incoming packet
    int rxPacketSizeReceived = 0;   //!< Byte(s) of the incoming packet received from the serial link

    /*!
     * The software lock used to lock the serial interface, to avoid concurent
     * reads/writes that would lead to multiplexing and packet corruptions.
     * We need one lock per HerkuleX (or HerkuleXController) instance because
     * we want to keep the ability to use multiple serial interface simultaneously
     * (ex: /dev/tty0 and /dev/ttyUSB0).
     */
    int commLock = 0;
    int commStatus = COMM_RXSUCCESS;//!< Last communication status

    // Serial communication methods, using one of the SerialPort[Linux/Mac/Windows] implementations.
    void hkx_tx_packet();
    void hkx_rx_packet();
    void hkx_txrx_packet(int ack);

protected:
    HerkuleX();
    virtual ~HerkuleX() = 0;

    int serialDevice= SERIAL_UNKNOWN;   //!< Serial device in use (if known) using '::SerialDevices_e' enum. Can affect link speed and latency.
    int servoSerie = SERVO_DRS;         //!< Servo serie using '::ServoDevices_e' enum. Used internally to setup some parameters like maxID, ackPolicy and protocolVersion.

    int protocolVersion = PROTOCOL_HKX; //!< Version of the communication protocol in use.
    int maxId = 253;                    //!< Store in the maximum value for servo IDs.
    int ackPolicy = ACK_REPLY_READ;     //!< Set the status/ack packet return policy using '::AckPolicy_e' (0: No return; 1: Return for READ commands; 2: Return for all commands).

    // Handle serial link
    ////////////////////////////////////////////////////////////////////////////

    /*!
     * \brief Open a serial link with the given parameters.
     * \param devicePath: The path to the serial device node.
     * \param baud: The baudrate or HerkuleX 'baudnum'.
     * \return 1 if success, 0 if locked, -1 otherwise.
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
    int hkx_get_com_status();       //!< Get communication status (commStatus) of the latest TX/RX instruction
    int hkx_get_com_error();        //!< Get communication error (if commStatus is an error) of the latest TX/RX instruction
    int hkx_get_com_error_count();  //!< 1 if commStatus is an error, 0 otherwise
    int hkx_print_error();          //!< Print the last communication error
    int hkx_print_status();         //!< Print the current status
    void printRxPacket();           //!< Print the RX buffer (last packet received)
    void printTxPacket();           //!< Print the TX buffer (last packet sent)

    // Instructions
    bool hkx_ping(const int id, PingResponse *status = nullptr, const int ack = ACK_DEFAULT);
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

#endif // HERKULEX_H
