SmartServoFramework 0.95
========================

[![Build Status](https://travis-ci.org/emericg/SmartServoFramework.svg?branch=master)](https://travis-ci.org/emericg/SmartServoFramework)

![SERVO](https://raw.githubusercontent.com/emericg/SmartServoFramework/master/gui/resources/img/dynamixel_ax12_diagram.png)

## Introduction

SmartServoFramework is a C++ multi-platform framework used to drive "smart servo" devices like Dynamixel or HerkuleX actuators. It has been developed at Inria Grenoble research center.

**Linux** (and most Unix systems), **Mac OS X** and **Windows** operating systems are supported. All you need to begin moving stuff around is at least one servo, a serial port adapter and a modern C++ compiler! SmartServoFramework works well on Raspberry Pi or other embedded boards, but will not work on Arduinos nor any other microcontrollers.

> Dynamixel devices from [Robotis](http://www.robotis.com/) and HerkuleX devices from [Dongbu Robot](http://www.dongburobot.com/) are high-performance networked actuators for robots available in wide range of sizes and strengths.  
> They have adjustable torque, speed, angle limits, and provide various feedback like position, load, voltage and temperature...

This framework **can be used with any Dynamixel or HerkuleX devices**. That means AX series, MX "T" series, XL-320, DRS-0101, and the others! Please note that the Dynamixel PRO devices *should* work (the dynamixel protocol v2 is implemented, and works great with XL-320 devices), but Dynamixel PRO devices are marked as not supported as we couldn't test with any real device.  
Support for other devices or brands may be added in the futur...

* **Simple API:** Use this API to easily get/set values to your servos by sending simple synchronous instructions, then waiting for the answers!  
* **Managed API:** Setup a controller and attach servo instances to it. Manipulate servo objects and let the controller synchronize its "virtual" register values with the real servo hardware in a background thread with a fixed frequency.  Beware: this API is more complex to master, and not entirely stable ;-)

### API documentation

You can dynamically generate an API documentation from the source code using Doxygen. The documentation will be generated inside the `doc/` directory.
If you are running a Mac/Linux system you can easily generate the documentation from a terminal:
> $ cd SmartServoFramework/  
> $ doxygen Doxyfile  

If you are allergic to the command line, it's just as easy to generate the documentation with DoxyWizard, Doxygen's GUI.

## Using the framework

### Dependencies

You will need a modern C++11 capable compiler:
* GCC >= 4.6
* LLVM >= 3.0
* MSVC >= 2012

Build system:
* CMake (**ONLY** to build the standalone library)
* Scons (**ONLY** to build the examples)
* Doxygen (**ONLY** to generate the documentation)

Using Linux?
* liblockdev (used to lock the serial port to a single software instance, which will save you from a lot of potential head scratching errors...)

### Serial link

This framework can be used with any combination of RS-232 ports, USB to TTL adapters, USB to RS-485 adapters, half or full duplex... But you'll need the right link for the right device.

First make sure that you can access your serial port:
* If you are running Linux, you will need special permissions from the `uucp` and/or `dialout` groups in order to access serial ports. You can add your user account to these groups with this command: `# useradd -G uucp,dialout $USER` (you'll need root credentials for this operation).
* If you are running Mac OS X, depending on your adapter, you may need to install the [FTDI driver](http://www.robotis.com/xe/download_en/646927), or the [CP210x driver](http://www.silabs.com/products/mcu/pages/usbtouartbridgevcpdrivers.aspx).
* If you are running Windows, you will need to install the [FTDI driver for the USB2Dynamixel device](http://www.robotis.com/xe/download_en/646927). You may also need other drivers depending on your adapter (like the [USB2AX driver](https://raw.githubusercontent.com/Xevel/usb2ax/master/firmware/lufa_usb2ax/USB2AX.inf), the [CP210x driver](http://www.silabs.com/products/mcu/pages/usbtouartbridgevcpdrivers.aspx), or the official [FTDI driver](http://www.ftdichip.com/Drivers/D2XX.htm)).

Latency over the serial port will limit the number of instructions you can send each second even more than bandwidth limitations.
A few tips to minimize traffic on your serial port:  
- You can use [these tips](https://projectgus.com/2011/10/notes-on-ftdi-latency-with-arduino/) to reduce latency on serial ports adapter using FTDI chips!  
- If you are using Dynamixel devices, you may want to reduce the "Return Delay Time" value to a minimum, from the default of '250' to something like '25'. Check what value works for you.  
- For both Dynamixel and HerkuleX devices, you can set the "Status Return Level" / "Ack Policy" to '1' in order to minimize the number of status packets (only if you do not need them), or even '2' to disable them ALL. Check your servo manual for more info on this.  

One more important thing: you need to power your servos with **proper power supply**. Shaky power sources have been known to cause interferences on serial bus, resulting in numerous packet corruptions. Be careful when using batteries and power converters!

#### Serial link (Dynamixel servos)

> Note: Regular "full-duplex" TTL converters will NOT work with "half-duplex TTL" Dynamixel servos (AX series, MX "T" series, XL-320, ...).

* [USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax): Unofficial but awesome device designed to manage TTL communication with your Dynamixels (AX series, MX "T" series, XL-320, ...).
* [USB2Dynamixel](http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm): Official device that can manage regular RS232, RS485 and TTL communications.  
* Home made TTL half-duplex device: [like this setup](http://savageelectronics.blogspot.fr/2011/01/arduino-y-dynamixel-ax-12.html) from Savage Electronics (in spanish), or [this one](http://www.memememememememe.me/the-dynamixel/) from memememe (in english).

#### Serial link (HerkuleX servos)

You need a serial port with a regular "full-duplex" TTL converter to use HerkuleX devices.

### Building SmartServoFramework library

> $ cd SmartServoFramework/build/  
> $ cmake ..  
> $ make  

The shared library will be located inside the `build/` directory.

### Building test softwares

Various test softwares are available in the `examples/` directory for you to play with:

* ex_simple: Control up to four servos with your keyboard using the 'Simple API'.  
* ex_simple_threaded: ex_simple variant, with servos actions and keyboard handled in two separate threads.  
* ex_controller: Control four servos with your keyboard using the 'Managed API'.  
* ex_sinus_control: Control a servo with sinusoid curve for both speed and position. Enable OpenCV to get a nice position/speed graph.  
* ex_advance_scanner: Scan serial ports for Dynamixel servos, for all IDs and all (but configurable) serial port speeds.  

You can build them all at once:
> $ cd SmartServoFramework/examples  
> $ scons  

The executables will be located inside the `examples/build/` directory.

## SmartServoGui

SmartServoGui is a fully featured Qt GUI application that helps you discover devices on available serial links, get an overview of all of their registers, and easily changes their settings!

SmartServoGui has its own README int the `gui/` directory, where you can learn more about its features!

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
