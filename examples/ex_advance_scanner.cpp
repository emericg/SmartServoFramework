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
 * \file ex_advance_scanner.cpp
 * \date 27/05/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 */

/*
 * Scan serial ports for Dynamixel servos, for all IDs ([0:253]) and all serial port speeds.
 * You can choose what serial speed you want to use for the scan through '-start' and '-stop'
 * arguments. Note: baudrate(BPS) = 2000000 / (Data + 1)
 *
 * HerkuleX devices are not supported by this example, please use the SmartServoGUI
 * for a more exaustive search software.
 */

// Smart Servo Framework
#include "../src/DynamixelSimpleAPI.h"

// C++ standard library
#include <iostream>
#include <vector>
#include <map>
#include <cstring>

/* ************************************************************************** */

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======== Smart Servo Framework Tester ========" << std::endl;

    int start = 0, stop = 253;

    // Argument(s) parsing
    if (argc == 1)
    {
        std::cerr << "ex_advance_scanner: no argument! Using default [0:253] ID scanning range." << std::endl;
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            //std::cout << "argv[" << i << "] = " << argv[i] << std::endl;

            if (strncmp(argv[i], "-start", sizeof("-start")) == 0)
            {
                if (argv[i+1] != NULL)
                {
                    start = std::atoi(argv[i+1]);

                    if (start < 0 || start > 252)
                    {
                        start = 0;
                    }
                }
                else
                {
                    std::cerr << "-start: value out of boundaries" << std::endl;
                }
            }
            else if (strncmp(argv[i], "-stop", sizeof("-stop")) == 0)
            {
                if (argv[i+1] != NULL)
                {
                    stop = std::atoi(argv[i+1]);

                    if (stop < 1 || stop > 253)
                    {
                        stop = 253;
                    }
                }
                else
                {
                    std::cerr << "-stop: value out of boundaries" << std::endl;
                }
            }
        }
    }

    std::cout << std::endl << "======== Advance Scanner ========" << std::endl;

    std::multimap <int, int> search_results;

    // Initialize a Dynamixel "Simple API" instance
    DynamixelSimpleAPI dxl;

    // Try a new on all 'bandnums'
    for (int i = start; i <= stop; i++)
    {
        // Initialize a serial link for the controller with serial port auto-detection
        std::string deviceName = "auto";
        if (dxl.serialInitialize(deviceName, i) == 0)
        {
            std::cerr << "> Failed to open a serial link for our SimpleAPI!" << std::endl;
            break;
        }

        // Scan serial port for servos, store results inside a vector
        std::vector <int> ids = dxl.servoScan();

        // Add results to global search results
        if (ids.size() > 0)
        {
            for (auto id: ids)
            {
                search_results.insert(std::pair<int, int>(dxl_get_baudrate(i), id));
            }
        }

        // Close device(s)
        dxl.serialTerminate();
    }

    std::cout << std::endl << "======== SEARCH RESULTS ========" << std::endl;

    std::cout << "Search results:\n";
    for (auto it: search_results)
    {
        std::cout << it.first << " => " << it.second << '\n';
    }

    std::cout << std::endl << "======== EXITING ========" << std::endl;

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
