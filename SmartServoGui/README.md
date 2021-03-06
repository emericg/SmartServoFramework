SmartServoGui
=============

SmartServoGui is a fully featured Qt GUI application that helps you discover devices on available serial links, get an overview of all of their registers, and easily tweak their settings!

**Features:**
* Works with virtually any Dynamixel and HerkuleX devices!
* Works on Linux, macOS and Windows
* Works with any number of serial ports, each on its own protocol (dxl v1/v2 or hkx)
* Visualize and change every servo registers
* Check out your servo characteristic and access the manuals
* Use the advance scanner to find your lost devices! Very useful when you mess up with an ID or a baudrate.

## Screenshots!

![GUI1](http://i.imgur.com/9mkQFUx.png)

![GUI2](http://i.imgur.com/x3sXE31.png)

![GUI2](http://i.imgur.com/8iBKiKN.png)

![GUI3](http://i.imgur.com/bE2qYIk.png)

![GUI3](http://i.imgur.com/XZt7WsF.png)

## Instructions

### Dependencies

You will need a modern C++11 capable compiler:
* GCC >= 5.1  
* LLVM >= 4.0  
* MSVC >= 2015  

Libraries:
* Qt 5.6+  
* SmartServoFramework (we got this one covered)  

### Hardware

Yes, it works better if you have a serial port adapter and a servo :-)

### Building SmartServoGui

> $ cd SmartServoFramework/SmartServoGui/  
> $ qmake  
> $ make  

## Get involved!

### Users

You can help us finding bugs, proposing new functionalities and more directly on this website! Click on the "New issue" button in the menu to do that.

### Developers

You can browse the git repository here on GitHub, submit patches and push requests!

## Licensing

SmartServoFramework is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the licence on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

Copyright (c) 2014, INRIA, All rights reserved.  
Emeric Grange <emeric.grange@gmail.com>  
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr>  
