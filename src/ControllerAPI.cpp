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
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

#include "ControllerAPI.h"

// C standard library
#include <string.h>

// C++ standard libraries
#include <iostream>
#include <chrono>
#include <thread>

/* ************************************************************************** */

ControllerAPI::ControllerAPI(int freq):
    controllerState(state_stopped),
    syncloopRunning(false),
    syncloopCounter(0),
    errors(0)
{
    if (freq < 1 || freq > 120)
    {
        syncloopFrequency = 30;
        syncloopDuration = 1000.0 / 30.0;
    }
    else
    {
        syncloopFrequency = freq;
        syncloopDuration = 1000.0 / static_cast<double>(freq);
    }
}

ControllerAPI::~ControllerAPI()
{
    //
}

/* ************************************************************************** */

void ControllerAPI::startThread()
{
    if (syncloopRunning == false && syncloopThread.joinable() == 0)
    {
        errors = 0;
        syncloopRunning = true;
        syncloopThread = std::thread(&ControllerAPI::run, this);
        controllerState = state_started;
    }
}

void ControllerAPI::pauseThread_internal()
{
    if (syncloopRunning == true && syncloopThread.joinable() == 1)
    {
        syncloopRunning = false;
        syncloopThread.join();
        controllerState = state_paused;
    }
    else if (syncloopRunning == false && syncloopThread.joinable() == 0)
    {
        syncloopRunning = true;
        syncloopThread = std::thread(&ControllerAPI::run, this);
        controllerState = state_ready;
    }
}

void ControllerAPI::stopThread()
{
    unregisterServos_internal();
    clearMessageQueue();

    if (syncloopThread.joinable() == 1)
    {
        errors = 0;
        syncloopRunning = false;
        syncloopThread.join();
        controllerState = state_stopped;
    }
}

/* ************************************************************************** */

bool ControllerAPI::waitUntilReady()
{
    std::chrono::seconds timeout(static_cast<int>(6));
    std::chrono::milliseconds waittime(static_cast<int>(2));
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    // Wait until the controller is in scanned state, but not ready
    // because if there is no results and hence it will never be ready
    while (getState() < state_scanned)
    {
        // timeout ?
        if ((start + timeout) < std::chrono::system_clock::now())
        {
            return false;
        }

        std::this_thread::sleep_for(waittime);
    }

    // If we do have results after the scan, we want to wait for every device to be properly read
    if (getServos().size() > 0)
    {
        // Wait until the controller is in 'ready' state
        while (getState() < state_ready)
        {
            // timeout ?
            if ((start + timeout) < std::chrono::system_clock::now())
            {
                return false;
            }

            std::this_thread::sleep_for(waittime);
        }
    }

    return true;
}

/* ************************************************************************** */

void ControllerAPI::registerServo_internal(Servo *servo)
{
    if (controllerState != state_scanning)
    {
        // Lock servoList
        std::lock_guard <std::mutex> lock(servoListLock);

        // Check if the servo is already registered
        for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); it++)
        {
            if ((*it)->getId() == (*servo).getId())
            {
                std::cerr << "Unable to register servo #" << (*servo).getId() << ": already registered!" << std::endl;
                return;
            }
        }
        std::cerr << "Registering servo #" << (*servo).getId() << std::endl;

        // Add servo to the controller vector
        servoList.push_back(servo);

        // Mark it for an "initial read"
        updateList.push_back((*servo).getId());

        // Mark it for "sync"
        syncList.push_back((*servo).getId());
    }
    else
    {
        std::cerr << "Cannot register a device while scanning!" << std::endl;
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
            it++;
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
            it++;
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
            it++;
        }
    }

    if (servoList.empty() == true)
    {
        // No more device(s), nothing left to do, we set the controller state back to idle
        controllerState = state_started;
    }
}

void ControllerAPI::unregisterServos_internal()
{
    // Lock servoList
    std::lock_guard <std::mutex> lock(servoListLock);

    for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); it++)
    {
        delete *it;
    }

    errors = 0;

    servoList.clear();
    updateList.clear();
    syncList.clear();

    // No more device(s), nothing left to do, we set the controller state back to idle
    controllerState = state_started;
}

int ControllerAPI::delayedAddServos_internal(std::chrono::time_point <std::chrono::system_clock> delay, int id, int update)
{
    if (delay < std::chrono::system_clock::now())
    {
        std::cout << "Adding back servo #" << id << " to its controller" << std::endl;
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

    for (std::vector <Servo *>::iterator it = servoList.begin(); it != servoList.end(); it++)
    {
        if ((*it)->getId() == id)
        {
            return *it;
        }
    }

    return NULL;
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
    if (syncloopRunning == true)
    {
        //std::cout << "> sendMiniMessage() " << std::endl;

        std::lock_guard <std::mutex> lock(m_mutex);
        m_queue.push_back(*m);
    }
    else
    {
        //std::cerr << "> sendMiniMessage() error: thread not running" << std::endl;
    }
}

void ControllerAPI::clearMessageQueue()
{
    m_mutex.lock();
    m_queue.clear();
    m_mutex.unlock();
}

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

void ControllerAPI::pauseThread()
{/*
    miniMessages m;
    memset(&m, 0, sizeof(m));

    m.msg = ctrl_pause;

    sendMessage(&m);
*/
    pauseThread_internal();
}

/* ************************************************************************** */
