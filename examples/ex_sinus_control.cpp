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
 * \file ex_sinus_control.cpp
 * \date 28/04/2014
 * \author Emeric Grange <emeric.grange@inria.fr>
 *
 * Control one servo with sinusoid curve for speed and position.
 * OpenCV can be used to visualize the position and speed curve of the servo.
 */

#define ENABLE_OPENCV_VIZ       1
#define SYNC_FREQUENCY         30
#define SOFTWARE_FREQUENCY     30

// SmartServoFramework
#include "../src/DynamixelController.h"
#include "../src/HerkuleXController.h"

// C++ standard libraries
#include <iostream>
#include <cmath>

#if ENABLE_OPENCV_VIZ == 1
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

/* ************************************************************************** */

// ID of the servos you want to use in this test program:
#define ID_SERVO      1

/* ************************************************************************** */

int main(int argc, char *argv[])
{
    std::cout << std::endl << "======== Smart Servo Framework Tester ========" << std::endl;

    // Initialize a DynamixelController instance
    DynamixelController ctrl(SYNC_FREQUENCY);

    // Initialize a serial link for the controller
    // You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
    // Note: serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.
    std::string deviceName = "auto";
    if (ctrl.connect(deviceName, 1000000) == 0)
    {
        std::cerr << "> Failed to open a serial link for our ControllerAPI! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << std::endl << "======== Setting initial parameters ========" << std::endl;

    int oldpos = 0;
    int t = 0;   // ticks
    int w = 800; // half the number of steps of amplitude // also, opencv window size

#if ENABLE_OPENCV_VIZ == 1
    std::cout << std::endl << "======== Setting up OpenCV ========" << std::endl;

    // Window name
    char window[] = "Sinus Control Visualization";

    // Create black empty image
    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

    // Legend
    cv::Scalar cp(0, 255, 0);
    cv::Scalar ccp(0, 0, 255);
    cv::Scalar cs(255, 0, 0);
    cv::Scalar ccs(128, 128, 0);
#endif /* ENABLE_OPENCV_VIZ */

    std::cout << std::endl << "======== Setting up servo ========" << std::endl;

    ServoAX servo(ID_SERVO, SERVO_AX12A, SPEED_MANUAL);
    ctrl.registerServo(&servo);

    // Wait for the controller to register our servos
    ctrl.waitUntilReady();

    servo.setMovingSpeed(512);

    std::cout << std::endl << "======== MAIN LOOP ========" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    double syncloopDuration = 1000.0 / static_cast<double>(SOFTWARE_FREQUENCY);
    bool running = true;

    while (running)
    {
        start = std::chrono::system_clock::now();

        // Maths:
        // x(t) = pos   = a * sin(b * t) + offset;
        // v(t) = speed = dx(t) / dt = a * b * cos(b * t);
        // Vcmd = v(t) + k  * (x(t) - xmes);
        ///////////////////////////////////////////////////////////////////////

        double k = 1.0; // correction factor // diff between current and expected pos
        double b = (1.0/static_cast<double>(w)) * 2.0 * 3.14;
        double pos = sin(b * static_cast<double>(t)) * w/2.0 + w/2.0;
        double speed = abs(2.0 * (static_cast<double>(w)/2.0 * b * cos(b * static_cast<double>(t)))) * 8.0;
        //std::cout << "[Pos: " << pos << " Speed: " << speed << "]" << std::endl;

        // Servo orders
        ///////////////////////////////////////////////////////////////////////

        double currpos = servo.getCurrentPosition();
        int currspeed = servo.getCurrentSpeed();

        if (abs(pos - currpos) > 1.0)
        {
            if (pos < oldpos)
                k = currpos / pos;
            else
                k = pos / currpos;

            if (k < 0.0 || k > 5.0)
                k = 1.0;
        }

        int sendspeed = speed * k + 1.0;
        int sendpos = (pos > oldpos) ? 1023 : 0;
        servo.setMovingSpeed(sendspeed);
        servo.setGoalPosition(sendpos);
        //std::cout << "[Pos: " << pos << "  |  Speed: " << speed << "  |  k: " << k << "]" << std::endl;
        //std::cout << "[S Pos: " << sendpos << "  |  Speed: " << sendspeed << "  |  k: " << k << "]" << std::endl;
        //std::cout << "[C Pos: " << pos << "  |  C Speed: " << speed << "]" << std::endl;

#if ENABLE_OPENCV_VIZ == 1
        // Viz
        ///////////////////////////////////////////////////////////////////////

        if (t == 0)
        {
            // Re-draw background, scale and legends
            img = cv::Scalar(48, 48, 48);
            cv::line(img, cv::Point(w, w/2), cv::Point(0, w/2), cv::Scalar(255, 255, 255));
            putText(img, "Position: ", cv::Point(16, 32),2, 1, cv::Scalar(200, 200, 200));
            putText(img, "expected", cv::Point(160, 32),2, 1, cp);
            putText(img, "/ current", cv::Point(305, 32),2, 1, ccp);
            putText(img, "Speed: ", cv::Point(16, 64),2, 1, cv::Scalar(200, 200, 200));
            putText(img, "expected", cv::Point(132, 64),2, 1, cs);
            putText(img, "/ current", cv::Point(280, 64),2, 1, ccs);
        }

        cv::Point pp(t, pos);
        cv::line(img, pp, pp, cp, 2);

        cv::Point pcp(t, currpos);
        cv::line(img, pcp, pcp, ccp, 2);

        cv::Point ps(t, -speed + w/2);
        cv::line(img, ps, ps, cs, 2);

        cv::Point cps(t, -(currspeed % 1024) + w/2);
        cv::line(img, cps, cps, ccs, 2);

        cv::imshow(window, img);
#endif /* ENABLE_OPENCV_VIZ */

        // Loop control (No need to go faster than the sync thread)
        ///////////////////////////////////////////////////////////////////////

        oldpos = pos;

        (t > w) ? t = 0 : t++;
        end = std::chrono::system_clock::now();
        double loopd = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        double waitd = (syncloopDuration*1000.0) - loopd;

#if ENABLE_OPENCV_VIZ == 1
        if (waitd >= 1000.0)
        {
            cv::waitKey(static_cast<int>(waitd/1000.0));
        }
        else
        {
            cv::waitKey(1);
        }
#else /* ENABLE_OPENCV_VIZ == 0*/
        if (waitd > 0.0)
        {
            std::chrono::microseconds waittime(static_cast<int>(waitd));
            std::this_thread::sleep_for(waittime);
        }
#endif
    }

    // Stop controller and close serial device(s)
    ctrl.disconnect();

    return EXIT_SUCCESS;
}

/* ************************************************************************** */
