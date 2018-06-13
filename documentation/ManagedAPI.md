Managed API
-----------

> Setup a controller and attach servo instances to it. Manipulate servo objects and let the controller synchronize (at a fixed frequency) its "virtual" register values with the real servo devices using a background thread. Beware: this API is more complex to master, and not entirely stable yet ;-)

### Advantages

- Asynchronous API
- Easily handle multiple servo models on the same bus, each servo can have its own settings
- Validate each values sent or read from servos

### Limitations

- Higher level API, you might loose some control on low level communication

## Setup walkthrough

First, you need to include the Managed API header file:
> #include <SmartServoFramework/ManagedAPI.h>  

Then initialize a Dynamixel (or HerkuleX) "controller" instance:
> DynamixelController ctrl;  

Connect your controller instance to a serial port:
> std::string serialDevicePath = "auto";  
> int serialDeviceBaudrate = 1000000;  
> if (ctrl.connect(serialDevicePath, serialDeviceBaudrate) == 0)  
> {  
>    std::cerr << "> Failed to open a serial link for your controller instance!" << std::endl;  
> }  

Notes:
* You can specify the serial port path directly if you know it. Ex: "/dev/ttyUSB0" for a Linux system; "//./COM1" for a Windows system.
* Serial port "auto-detection" will only work if a single serial port adapter is connected to your computer, or if the fisrt one detected is the one connected to your devices.

The next step is to instanciate servos and register them to your controller:
> ServoAX s1(ID_SERVO_1, SERVO_AX12W);  
> ServoMX s2(ID_SERVO_2, SERVO_MX64);  
> ctrl.registerServo(&s1);  
> ctrl.registerServo(&s2);  
> ctrl.waitUntilReady();  

Note:
* The managed API can mix different servo models with very different settings. The only thing you can't do is mixing servo with different protocols (Dynamixel v1 / v2 or HerkuleX protocols).

Then you can just read and write values to your servos!
> int pos = s1.getCurrentPosition();  
> s1.setGoalPosition(512);  

Once you're done, you'll just need to disconnect the controller:
> ctrl.disconnect();  

