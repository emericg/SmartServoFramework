API breakage
------------

version 0.98
- You only need to include <SmartServoFramework/SimpleAPI.h> or <SmartServoFramework/ManagedAPI.h> directly in your project now
- ControllerAPI class (and file) is now ServoController to avoid confusion with the new ManagedAPI.h file
- Some directories and files have been shifted around

version 0.97
- DynamixelController() and HerkuleXController() params order have changed (now in line with the SimpleAPI)
- GUI settings folder has changed
- library name is not lowercase (smartservoframework)

version 0.96
- Needs more recent compiler (more C++11)
- Needs Qt5 for the SmartServoGui

