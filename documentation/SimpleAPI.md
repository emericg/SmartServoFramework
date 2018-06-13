Simple API
----------

> Use this API to easily get/set values to your servos by sending simple synchronous instructions, then waiting for the answers!

### Advantages

- Synchronous API: VERY easy to use
- Basic set of instruction common to all Dynamixel and HerkuleX devices
- Validities of values and results are checked
- Easily access unique registers using specific call + register name

### Limitations

- Synchronous API: application will block while waiting for servo responses
- Harder to efficiently handle multiple servo models on the same bus

## Setup walkthrough

First, you need to include the Simple API header file:
> #include <SmartServoFramework/SimpleAPI.h>  

Then initialize a Dynamixel (or HerkuleX) "Simple API" instance:
> DynamixelSimpleAPI dxl(SERVO_XL320);  

Note:
* If you know the servo type/serie you're using, specify it right away in the constructor. It will be used to set the correct protocol version, as well as selecting the right capabilities and value limits used through the conveniance functions.

Connect your SimpleAPI instance to a serial port:
> std::string serialDevicePath = "auto";  
> int serialDeviceBaudrate = 1000000;  
> if (dxl.connect(serialDevicePath, serialDeviceBaudrate) == 0)  
> {  
>    std::cerr << "> Failed to open a serial link for your SimpleAPI instance!" << std::endl;  
> }  

Notes:
* You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
* Serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.

Then you can just read and write values to your servos!
In this example 1 is the ID of our servo:
> int pos = dxl.readCurrentPosition(1);  
> dxl.setGoalPosition(1, pos + 64);  

Instead of using conveniance functions like setGoalPosition(), you can also set values directly with a register name:
> int pos = dxl.getSetting(1, REG_CURRENT_POSITION);  
> dxl.setSetting(1, REG_GOAL_POSITION, pos + 64);  

Once you're done, you'll just need to disconnect the API:
> dxl.disconnect();  

