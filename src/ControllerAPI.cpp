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
 * \file ControllerAPI.cpp
 * \date 25/08/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "ControllerAPI.h"
#include "minitraces.h"

// C standard library
#include <cstring>
#include <cstdio>

// C++ standard libraries
#include <chrono>
#include <thread>

/* ************************************************************************** */

ControllerAPI::ControllerAPI(int ctrlFrequency)
{
    if (ctrlFrequency < 1 || ctrlFrequency > 120)
    {
        syncloopFrequency = 30;
        syncloopDuration = 1000.0 / 30.0;
    }
    else
    {
        syncloopFrequency = ctrlFrequency;
        syncloopDuration = 1000.0 / static_cast<double>(ctrlFrequency);
    }
}

ControllerAPI::~ControllerAPI()
{
    //
}

/* ************************************************************************** */

void ControllerAPI::startThread()
{
    if (getState() < state_started && syncloopThread.joinable() == 0)
    {
        clearErrorCount();
        setState(state_started);
        syncloopThread = std::thread(&ControllerAPI::run, this);
    }
}

void ControllerAPI::pauseThread()
{
    int ctrlState = getState();

    if (ctrlState >= state_started && syncloopThread.joinable() == 1)
    {
        TRACE_INFO(CAPI, ">> Pausing thread (id: %i)...", syncloopThread.get_id());

        miniMessages m;
        memset(&m, 0, sizeof(m));
        m.msg = ctrl_state_pause;
        sendMessage(&m);

        syncloopThread.join();
        setState(state_paused);
    }
    else if (ctrlState == state_paused && syncloopThread.joinable() == 0)
    {
        TRACE_INFO(CAPI, ">> Unpausing thread (id: %i)...", syncloopThread.get_id());

        setState(state_ready);
        syncloopThread = std::thread(&ControllerAPI::run, this);
    }
    else
    {
        TRACE_ERROR(CAPI, ">> Cannot pause/unpause thread (id: %i): unknown controller state (%i)...", syncloopThread.get_id(), ctrlState);
    }
}

void ControllerAPI::stopThread()
{
    if (getState() != state_stopped && syncloopThread.joinable() == 1)
    {
        TRACE_INFO(CAPI, ">> Stopping thread (id: %i)...", syncloopThread.get_id());

        // Send termination message
        miniMessages m;
        memset(&m, 0, sizeof(m));
        m.msg = ctrl_state_stop;
        sendMessage(&m);

        // Wait for the thread to finish
        syncloopThread.join();

        // Cleanup controller
        unregisterServos_internal();
        clearMessageQueue();
        clearErrorCount();
        setState(state_stopped);
    }
}

/* ************************************************************************** */

void ControllerAPI::setState(const int state)
{
    if (state >= state_stopped && state <= state_ready)
    {
        std::lock_guard <std::mutex> lock(controllerStateLock);
        controllerState = state;
    }
    else
    {
        TRACE_ERROR(CAPI, ">> setState(%i) error: invalid state!", state);
    }
}

int ControllerAPI::getState()
{
    std::lock_guard <std::mutex> lock(controllerStateLock);
    return controllerState;
}

bool ControllerAPI::waitUntilReady()
{
    return waitUntil(state_started, 6);
}

bool ControllerAPI::waitUntil(int state, int timeout)
{
    // First, check if the controller is running, otherwise there is not point waiting for it to be ready...
    if (getState() < state)
    {
        TRACE_ERROR(CAPI, "waitUntilReady(): controller is not running!");
        return false;
    }

    std::chrono::seconds timeout_s(static_cast<int>(timeout));
    std::chrono::milliseconds wait_ms(static_cast<int>(2));
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    // Wait until the controller is at least in 'scanned' state, but not ready
    // because if there is no results and hence it will never be ready
    while (getState() < state_scanned)
    {
        if ((start + timeout_s) < std::chrono::system_clock::now())
        {
            TRACE_ERROR(CAPI, "waitUntilReady(): timeout!");
            return false;
        }

        std::this_thread::sleep_for(wait_ms);
    }

    // If we do have results after the scan, we want to wait for every device to be properly read
    if (getServos().size() > 0)
    {
        // Wait until the controller is in 'ready' state
        while (getState() < state)
        {
            if ((start + timeout_s) < std::chrono::system_clock::now())
            {
                TRACE_ERROR(CAPI, "waitUntilReady(): timeout!");
                return false;
            }

            std::this_thread::sleep_for(wait_ms);
        }
    }

    return true;
}

/* ************************************************************************** */

void ControllerAPI::updateErrorCount(int error)
{
    std::lock_guard <std::mutex> lock(errorCountLock);
    errorCount += error;
}

int ControllerAPI::getErrorCount()
{
    std::lock_guard <std::mutex> lock(errorCountLock);
    return errorCount;
}

/*!
 * \brief Reset error count for this controller.
 */
void ControllerAPI::clearErrorCount()
{
    std::lock_guard <std::mutex> lock(errorCountLock);
    errorCount = 0;
}

/* ************************************************************************** */

void ControllerAPI::registerServo_internal(Servo *servo)
{
    if (getState() >= state_started)
    {
        if (getState() != state_scanning)
        {
            // Lock servoList
            std::lock_guard <std::mutex> lock(servoListLock);

            // Check if the servo is already registered
            for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); ++it)
            {
                if ((*it)->getId() == (*servo).getId())
                {
                    TRACE_ERROR(CAPI, "Unable to register servo #%i: already registered!", (*servo).getId());
                    return;
                }
            }
            TRACE_INFO(CAPI, "Registering servo #%i", (*servo).getId());

            // Add servo to the controller vector
            servoList.push_back(servo);

            // Mark it for an "initial read"
            updateList.push_back((*servo).getId());

            // Mark it for "sync"
            syncList.push_back((*servo).getId());
        }
        else
        {
            TRACE_ERROR(CAPI, "Cannot register a device: controller is scanning!");
        }
    }
    else
    {
        TRACE_ERROR(CAPI, "Cannot register a device: controller is not running!");
    }
}

void ControllerAPI::unregisterServo_internal(Servo *servo)
{
    // Lock servoList
    std::lock_guard <std::mutex> lock(servoListLock);

    for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end();)
    {
        if ((*it)->getId() == (*servo).getId())
        {
            servoList.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (std::vector <int>::iterator it = updateList.begin(); it != updateList.end();)
    {
        if (*it == (*servo).getId())
        {
            updateList.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (std::vector <int>::iterator it = syncList.begin(); it != syncList.end();)
    {
        if (*it == (*servo).getId())
        {
            syncList.erase(it);
        }
        else
        {
            ++it;
        }
    }

    if (servoList.empty() == true)
    {
        // No more device(s), nothing left to do, we set the controller state back to idle
        setState(state_started);
    }
}

void ControllerAPI::unregisterServos_internal()
{
    // Lock servoList
    std::lock_guard <std::mutex> lock(servoListLock);

    for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); ++it)
    {
        delete *it;
    }

    clearErrorCount();

    servoList.clear();
    updateList.clear();
    syncList.clear();

    // No more device(s), nothing left to do, we set the controller state back to idle
    setState(state_started);
}

int ControllerAPI::delayedAddServos_internal(std::chrono::time_point <std::chrono::system_clock> delay, int id, int update)
{
    if (delay < std::chrono::system_clock::now())
    {
        TRACE_INFO(CAPI, "Adding back servo #%i to its controller", id);
        servoListLock.lock();

        syncList.push_back(id);
        if (update == 1)
        { updateList.push_back(id); }
        servoListLock.unlock();

        return 0;
    }
    else
    {
        return 1;
    }
}

/* ************************************************************************** */

Servo *ControllerAPI::getServo(const int id)
{
    // Lock servoList
    std::lock_guard <std::mutex> lock(servoListLock);

    for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); ++it)
    {
        if ((*it)->getId() == id)
        {
            return *it;
        }
    }

    return nullptr;
}

const std::vector <Servo *> ControllerAPI::getServos()
{
    // Lock servoList
    std::lock_guard <std::mutex> lock(servoListLock);

    // We only return a copy of the list
    const std::vector <Servo *> servos(servoList);
    return servos;
}

/* ************************************************************************** */

void ControllerAPI::sendMessage(miniMessages *m)
{
    if (getState() >= state_started)
    {
        TRACE_3(CAPI, "> sendMessage()");

        std::lock_guard <std::mutex> lock(m_mutex);
        m_queue.push_back(*m);
    }
    else
    {
        TRACE_ERROR(CAPI, "sendMiniMessage() error: controller's thread not running");
    }
}

void ControllerAPI::clearMessageQueue()
{
    m_mutex.lock();
    m_queue.clear();
    m_mutex.unlock();
}

/* ************************************************************************** */

void ControllerAPI::autodetect(int start, int stop)
{
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_device_autodetect;
    m.p1 = start;
    m.p2 = stop;

    sendMessage(&m);
}

void ControllerAPI::registerServo(Servo *servo)
{
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_device_register;
    m.p = (void *)(servo);

    sendMessage(&m);
}

void ControllerAPI::registerServo(int id)
{
/*
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_device_register;
    m.p = (void *)(servo);

    sendMessage(&m);
*/
}

void ControllerAPI::unregisterServo(Servo *servo)
{
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_device_unregister;
    m.p = (void *)(servo);

    sendMessage(&m);
}

void ControllerAPI::unregisterServo(int id)
{
/*
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_device_unregister;
    m.p = (void *)(servo);

    sendMessage(&m);
*/
}

/* ************************************************************************** */
