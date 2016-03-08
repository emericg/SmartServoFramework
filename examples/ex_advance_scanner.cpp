/*!
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 INRIA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * \file ex_advance_scanner.cpp
 * \date 27/05/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 *
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
        if (dxl.connect(deviceName, i) == 0)
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
        dxl.disconnect();
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
