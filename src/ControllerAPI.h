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
 * \file ControllerAPI.h
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#ifndef CONTROLLER_API_H
#define CONTROLLER_API_H

#include "Servo.h"
#include "Utils.h"

#include <vector>
#include <deque>
#include <thread>
#include <mutex>

/** \addtogroup ManagedAPIs
 *  @{
 */

/*!
 * \brief The controllerState enum.
 * \todo move that into the ControllerAPI
 */
enum controllerState_e
{
    state_stopped = 0,
    state_paused,
    state_started,
    state_scanning,
    state_scanned,
    state_reading,
    state_ready,
};

/*!
 * \brief The ControllerAPI abstract class, root of the ManagedAPI.
 *
 * A "controller" provide a high level API to handle several servos object at the
 * same time. A program must instanciate servos instances and register them to
 * a controller. Each servo object is synchronized with its hardware counterpart
 * by the run() method, running in its own backgound thread.
 */
class ControllerAPI
{
    int controllerState = 0;            //!< The current state of the controller, used by client apps to know.
    std::mutex controllerStateLock;     //!< Lock for the controllerState.

    int errorCount = 0;                 //!< Store the number of transmission errors.
    std::mutex errorCountLock;          //!< Lock for the error count.

protected:

    enum controllerMessage_e
    {
        ctrl_device_autodetect = 0,
        ctrl_device_register,
        ctrl_device_unregister,
        ctrl_device_unregister_all,
        ctrl_device_delayed_add,

        ctrl_state_pause,
        ctrl_state_stop,
    };

    struct miniMessages
    {
        controllerMessage_e msg;
        std::chrono::time_point <std::chrono::system_clock> delay; //!< Used to delay message parsing
        void *p;
        int p1;
        int p2;
    };

    int syncloopFrequency;              //!< Frequency of the synchronization loop, in Hz. May not be respected if there is too much traffic on the serial port.
    int syncloopCounter = 0;
    double syncloopDuration;            //!< Maximum duration for the synchronization loop, in milliseconds.

    std::thread syncloopThread;         //!< Controller's thread.

    std::deque<struct miniMessages> m_queue; //!< Message queue.
    std::mutex m_mutex;                 //!< Lock for the message queue.

    std::vector <Servo *> servoList;    //!< List containing device object managed by this controller.
    std::mutex servoListLock;           //!< Lock for the device list.

    std::vector <int> updateList;       //!< List of device object marked for a "full" register update.
    std::vector <int> syncList;         //!< List of device object to keep in sync.

    //! Read/write synchronization loop, running inside its own background thread
    virtual void run() = 0;

    /*!
     * \brief Internal thread messaging system.
     * \param m: A pointer to a miniMessages structure. Will be copied.
     *
     * Push a message into the back of a deque (if the thread is running, otherwise
     * messages will be discarded with an error).
     */
    void sendMessage(miniMessages *m);

    void registerServo_internal(Servo *servo);
    void unregisterServo_internal(Servo *servo);
    void unregisterServos_internal();
    int delayedAddServos_internal(std::chrono::time_point<std::chrono::system_clock> delay, int id, int update);
    virtual void autodetect_internal(int start = 0, int stop = 253) = 0;

    /*!
     * \brief Start synchronization loop thread.
     */
    void startThread();

    /*!
     * \brief Stop synchronization loop thread. All servos are deleted from its internal lists.
     */
    void stopThread();

    /*!
     * \brief Change the current state of the controller.
     */
    void setState(const int state);

    /*!
     * \brief Increment the number of error logged on the serial link associated with this controller.
     * \param error: Number of error to add to the counter.
     */
    void updateErrorCount(int error);

public:
    /*!
     * \brief ControllerAPI constructor.
     * \param ctrlFrequency: This is the synchronization frequency between the controller and the servos devices. Range is [1;120].
     */
    ControllerAPI(int ctrlFrequency);

    /*!
     * \brief ControllerAPI destructor. Stop the controller's thread and close the serial connection.
     */
    virtual ~ControllerAPI();

    /*!
     * \brief Connect the controller to a serial port, if the connection is successfull start a synchronization thread.
     * \param devicePath: The serial port device path.
     * \param baud: The serial port speed, can be a baud rate or a 'baudnum'.
     * \param serialDevice: If known, the serial adapter model used by this link.
     * \return 1 if the connection is successfull, 0 otherwise.
     */
    virtual int connect(std::string &devicePath, const int baud, const int serialDevice = 0) = 0;

    /*!
     * \brief Stop the controller's thread, clean umessage queue, and close the serial connection.
     */
    virtual void disconnect() = 0;

    /*!
     * \brief Pause/un-pause synchronization loop thread.
     */
    void pauseThread();

    /*!
     * \brief Read the current state of the controller.
     * \return The current state of the controller.
     */
    int getState();

    /*!
     * \brief Scan a serial link for servo or sensor devices.
     * \param start: First ID to be scanned.
     * \param stop: Last ID to be scanned.
     *
     * This scanning function will ping every device ID (from 'start' to 'stop',
     * default [0;253]) on a serial link, and use the status response to detect
     * the presence of a device.
     * When a device is being scanned, its LED is briefly switched on.
     *
     * Every servo found will be automatically registered to this controller.
     */
    void autodetect(int start = 0, int stop = 253);

    /*!
     * \brief Wait until the controller is ready.
     * \return true if the controller is ready, false if timeout has been hit and the controller status is unknown.
     *
     * You need to call this function after an autodetection or registering servos
     * manually, to let the controller some time to process new devices.
     * This is a blocking function that only return after the controller has been
     * proven ready, or a custom duration timeout hit.
     */
    bool waitUntilReady();
    bool waitUntil(int state, int timeout = 4);

    virtual std::string serialGetCurrentDevice_wrapper() = 0;
    virtual std::vector <std::string> serialGetAvailableDevices_wrapper() = 0;
    virtual void serialSetLatency_wrapper(int latency) = 0;

    /*!
     * \brief clearMessageQueue
     */
    void clearMessageQueue();

    /*!
     * \brief Return the number of error logged on the serial link associated with this controller.
     */
    int getErrorCount();

    /*!
     * \brief Reset error count for this controller.
     */
    void clearErrorCount();

    /*!
     * \brief Register a servo given in argument.
     * \param servo: A servo instance.
     *
     * Register the servo given in argument. It will be added to the controller
     * and thus added to the "synchronization loop".
     */
    void registerServo(Servo *servo);

    /*!
     * \brief Create and register a servo with the ID given in argument.
     * \param id: A servo id.
     *
     * Register the servo given in argument. It will be added to the controller
     * and thus added to the "synchronization loop".
     */
    void registerServo(int id);

    /*!
     * \brief Unregister a servo given in argument.
     * \param servo: A servo instance.
     */
    void unregisterServo(Servo *servo);

    /*!
     * \brief Unregister a servo with the ID given in argument.
     * \param id: A servo id.
     */
    void unregisterServo(int id);

    /*!
     * \brief Return a servo instance corresponding to the id given in argument.
     * \param id: The ID of the servo we want.
     * \return A servo instance.
     */
    Servo *getServo(const int id);

    /*!
     * \brief Return all servo instances registered to this controller.
     * \return A read only list of servo instances registered to this controller.
     */
    const std::vector <Servo *> getServos();
};

/** @}*/

#endif // CONTROLLER_API_H
