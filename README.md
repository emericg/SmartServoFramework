SmartServoFramework 0.96
========================

[![Build Status](https://travis-ci.org/emericg/SmartServoFramework.svg?branch=master)](https://travis-ci.org/emericg/SmartServoFramework)
[![Build status](https://ci.appveyor.com/api/projects/status/doqnmp6jrlqjyt22?svg=true)](https://ci.appveyor.com/project/emericg/smartservoframework)
[![License: LGPL v3](https://img.shields.io/badge/license-LGPL%20v3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0)

![SERVO](https://raw.githubusercontent.com/emericg/SmartServoFramework/master/gui/resources/img/dynamixel_ax12_diagram.png)

## Introduction

SmartServoFramework is a C++ multi-platform framework used to drive "smart servo" devices like Dynamixel or HerkuleX actuators. It has been developed at Inria Grenoble research center.

**Linux** (and most Unix systems), **macOS** and **Windows** operating systems are supported. All you need to begin moving stuff around is at least one servo, a serial port adapter and a modern C++ compiler! SmartServoFramework works well on Raspberry Pi or other embedded boards, but will not work on Arduinos nor any other microcontrollers.

> Dynamixel devices from [Robotis](http://www.robotis.com/) and HerkuleX devices from [Dongbu Robot](http://www.dongburobot.com/) are high-performance networked actuators for robots available in wide range of sizes and strengths.  
> They have adjustable torque, speed, angle limits, and provide various feedback like position, load, voltage and temperature...

This framework **can be used with any Dynamixel or HerkuleX devices**. We support all models of HerkuleX from the DRS-0101 up to the DRS-0602. For Dynamixel that means AX, EX, MX-T series, XL-320 models, X series, as well as the older RX and DX series. We even have support for the AX-S1 and IR sensor array! Please note that the Dynamixel PRO devices *should* work (the version 2 of the communication protocol is implemented, and works great with XL-320 devices), but they are considered as experimental since we couldn't test with any real device.  
Support for other devices, brands or protocols may be added in the futur...

We provide two different APIs:
* **Simple API:** Use this API to easily get/set values to your servos by sending simple synchronous instructions, then waiting for the answers!  
* **Managed API:** Setup a controller and attach servo instances to it. Manipulate servo objects and let the controller synchronize its "virtual" register values with the real servo hardware in a background thread with a fixed frequency.  Beware: this API is more complex to master, and not entirely stable ;-)  

## Documentation

> More informations on how to use the framework and your devices are available inside the `doc/` directory, and mirrored on the wiki. Please check these docs ;-)

You can dynamically generate a **full API documentation** from the source code using Doxygen. This will prove very important in order to work efficiently with the framework. The documentation will be accessible through `doc/API_documentation.html`.

If you are running a macOS/Linux system you can easily generate the documentation from a terminal:
> $ cd SmartServoFramework/  
> $ doxygen Doxyfile  

If you have problems with the command line, it's just as easy to generate the documentation using DoxyWizard, Doxygen's GUI.

## Building the framework

> A full documentation is available in the `doc/BuildInstructions.md`

### Dependencies

You will need a modern C++11 capable compiler:
* GCC >= 4.8  
* LLVM >= 3.6  
* MSVC >= 2015  

Build system:
* CMake (**ONLY** needed to build the standalone library)  
* Scons (**ONLY** needed to build the examples)  
* Doxygen (**ONLY** needed to generate the documentation)  

### Building SmartServoFramework library

> $ cd SmartServoFramework/build/  
> $ cmake ..  
> $ make  

### Building test softwares

Various test softwares are available in the `examples/` directory for you to play with:

* ex_simple: Control up to four servos with your keyboard using the 'Simple API'.  
* ex_simple_threaded: ex_simple variant, with servos actions and keyboard handled in two separate threads.  
* ex_controller: Control four servos with your keyboard using the 'Managed API'.  
* ex_sinus_control: Control a servo with sinusoid curve for both speed and position. Enable OpenCV to get a nice position/speed graph.  
* ex_advance_scanner: Scan serial ports for Dynamixel servos, for all IDs and all (but configurable) serial port speeds.  

You can build them all at once:
> $ cd SmartServoFramework/examples/  
> $ scons  

### Serial communication

> A full documentation is available in the `doc/SerialCommunication.md`

This framework can be used with any combination of RS-232 ports, USB to TTL adapters, USB to RS-485 adapters, half or full duplex... But you'll need the right link for the right device.

One more important thing: you need to power your servos with **proper power supply**. Shaky power sources have been known to cause interferences on serial bus, resulting in numerous packet corruptions. Be careful when using batteries and power converters!

First you will need to make sure your software can access your serial port:
* If you are running Linux, you may need special permissions from the `uucp` and/or `dialout` groups in order to access serial ports. You can add your user account to these groups with this command: `# useradd -G uucp,dialout $USER` (you'll need root credentials for this operation).
* If you are running macOS, depending on your adapter, you may need to install the [FTDI driver](http://www.robotis.com/xe/download_en/646927), or the [CP210x driver](http://www.silabs.com/products/mcu/pages/usbtouartbridgevcpdrivers.aspx).
* If you are running Windows, you may need to install the [FTDI driver for the USB2Dynamixel device](http://www.robotis.com/xe/download_en/646927). You may also need other drivers depending on your adapter (like the [USB2AX driver](https://raw.githubusercontent.com/Xevel/usb2ax/master/firmware/lufa_usb2ax/USB2AX.inf), the [CP210x driver](http://www.silabs.com/products/mcu/pages/usbtouartbridgevcpdrivers.aspx), or the official [FTDI driver](http://www.ftdichip.com/Drivers/D2XX.htm)).

#### Communication with Dynamixel devices

> Note: Regular "full-duplex" TTL converters will NOT work with "half-duplex TTL" Dynamixel servos (AX serie, MX "T" serie, XL-320, ...).

* [USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax): Unofficial but awesome device designed to manage TTL communication with your Dynamixels (AX serie, MX-T serie, XL-320, ...).  
* [USB2Dynamixel](http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm): Official device that can manage all Dynamixel devices through RS232 / RS485 / TTL communications.  
* Home made TTL half-duplex device: [like this setup](http://savageelectronics.blogspot.fr/2011/01/arduino-y-dynamixel-ax-12.html) from Savage Electronics (in spanish), or [this one](http://www.memememememememe.me/the-dynamixel/) from memememe (in english).  

#### Communication with HerkuleX devices

You need a serial port with a regular "full-duplex" TTL converter to use HerkuleX devices.

## SmartServoGui

> SmartServoGui has its own README in the `gui/` directory, where you can learn more about its features!

SmartServoGui is a fully featured Qt GUI application that helps you discover devices on available serial links, get an overview of all of their registers, and easily tweak their settings! Like the framework, it is fully multi-platform.

### Quick installation

> $ cd SmartServoFramework/gui/  
> $ qmake-qt${4-5}  
> $ make  
> $ ./build/SmartServoGui  

### Screenshots!

![GUI2](http://i.imgur.com/x3sXE31.png)

![GUI3](http://i.imgur.com/bE2qYIk.png)

## Get involved!

### Developers

You can browse the code here on GitHub, submit patches and pull requests! Your help would be greatly appreciated ;-)

### Users

You can help us finding bugs, proposing new features and more! Visit the "Issues" section in the GitHub menu to start.

## Licensing

SmartServoFramework is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the licence on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

Copyright (c) 2014, INRIA, All rights reserved.  
Emeric Grange <emeric.grange@gmail.com>  
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr>  
