# Project informations
###############################################################################

# General
project(SmartServoFramework)
cmake_minimum_required(VERSION 3.0)

# Pass various options to SmartServoFramework source code
###############################################################################

SET(SmartServoFramework_VERSION_MAJOR 0)
SET(SmartServoFramework_VERSION_MINOR 91)
SET(SmartServoFramework_VERSION_PATCH 0)

SET(ENABLE_DEBUG   1) # "Toggle debug traces"
SET(ENABLE_COLORS  1) # "Toggle colored terminal output"

# Doxygen module
FIND_PACKAGE(Doxygen) # FindDoxygen.cmake

IF(DOXYGEN_FOUND)
    MESSAGE(STATUS "* Doxygen found")
ELSE(DOXYGEN_FOUND)
    MESSAGE(WARNING " > Doxygen not found")
ENDIF(DOXYGEN_FOUND)

# Detect build type. You can change it using "-DCMAKE_BUILD_TYPE="
IF(CMAKE_BUILD_TYPE STREQUAL "Release")

    MESSAGE(STATUS "* This is a RELEASE build. You can change it using -DCMAKE_BUILD_TYPE=Debug.")
    SET(ENABLE_DEBUG 0)
    SET(CMAKE_BUILD_TYPE release)
    # Custom cflags? Use "release mode" and "-O2" for maximum speed.
    SET(CMAKE_CXX_FLAGS "-O2 -Wall -std=c++11 -Wno-unused -Wno-unused-parameter")

ELSE(CMAKE_BUILD_TYPE STREQUAL "Release")

    MESSAGE("!! * This is a DEBUG build. You can change it using -DCMAKE_BUILD_TYPE=Release.")
    SET(ENABLE_DEBUG 1)
    SET(CMAKE_BUILD_TYPE debug)
    # Custom cflags? Use "debug mode" and "-O0 -g" if you want better support for GDB and Valgrind.
    SET(CMAKE_CXX_FLAGS "-O0 -g -Wall -Wextra -std=c++11 -Wno-unused -Wno-unused-parameter")

ENDIF(CMAKE_BUILD_TYPE STREQUAL "Release")

## Save options into a "cmake_defines.h" file, using "cmake_defines.h.in" template
#CONFIGURE_FILE(
#    "${CMAKE_SOURCE_DIR}/src/cmake_defines.h.in"
#    "${CMAKE_SOURCE_DIR}/src/cmake_defines.h"
#)

# Source files
###############################################################################

SET(SmartServoFramework_SRC
    src/minitraces.cpp
    src/minitraces_conf.h
    src/minitraces.h
    src/ControllerAPI.cpp
    src/ControllerAPI.h
    src/ControlTables.cpp
    src/ControlTablesDynamixel.h
    src/ControlTables.h
    src/ControlTablesHerkuleX.h
    src/DynamixelController.cpp
    src/DynamixelController.h
    src/Dynamixel.cpp
    src/Dynamixel.h
    src/DynamixelSimpleAPI.cpp
    src/DynamixelSimpleAPI.h
    src/DynamixelTools.cpp
    src/DynamixelTools.h
    src/HerkuleXController.cpp
    src/HerkuleXController.h
    src/HerkuleX.cpp
    src/HerkuleX.h
    src/HerkuleXSimpleAPI.cpp
    src/HerkuleXSimpleAPI.h
    src/HerkuleXTools.cpp
    src/HerkuleXTools.h
    src/SerialPort.cpp
    src/SerialPort.h
    src/SerialPortLinux.cpp
    src/SerialPortLinux.h
    src/SerialPortMacOS.cpp
    src/SerialPortMacOS.h
    src/SerialPortWindows.cpp
    src/SerialPortWindows.h
    src/ServoAX.cpp
    src/ServoAX.h
    src/Servo.cpp
    src/ServoDRS.cpp
    src/ServoDRS.h
    src/ServoDynamixel.cpp
    src/ServoDynamixel.h
    src/ServoEX.cpp
    src/ServoEX.h
    src/Servo.h
    src/ServoHerkuleX.cpp
    src/ServoHerkuleX.h
    src/ServoMX.cpp
    src/ServoMX.h
    src/ServoXL.cpp
    src/ServoXL.h
    src/Utils.cpp
    src/Utils.h
)

# Build
###############################################################################

# Build library
ADD_LIBRARY(SmartServoFramework_shared SHARED ${SmartServoFramework_SRC})
#TARGET_LINK_LIBRARIES(SmartServoFramework_shared)
SET_TARGET_PROPERTIES(SmartServoFramework_shared PROPERTIES OUTPUT_NAME SmartServoFramework)

#IF(CMAKE_BUILD_MODE STREQUAL "Static")
#    MESSAGE("** Static library will be built")
#    ADD_LIBRARY(SmartServoFramework_static STATIC ${SmartServoFramework_SRC})
#    TARGET_LINK_LIBRARIES(SmartServoFramework_static ${JPEG_LIBRARY} ${PNG_LIBRARY})
#    SET_TARGET_PROPERTIES(SmartServoFramework_static PROPERTIES OUTPUT_NAME SmartServoFramework)
#ENDIF(CMAKE_BUILD_MODE STREQUAL "Static")

# Install the shared library and its header into the system (optional step, requires root credentials)
###############################################################################

# relative to $<INSTALL_PREFIX>
INSTALL(TARGETS SmartServoFramework_shared DESTINATION lib/)
INSTALL(DIRECTORY src/ DESTINATION include/SmartServoFramework FILES_MATCHING PATTERN "*.h")
