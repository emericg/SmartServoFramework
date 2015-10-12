# Project informations
###############################################################################

# General
project(SmartServoFramework)
cmake_minimum_required(VERSION 2.8)

# Pass various options to SmartServoFramework source code
###############################################################################

set(SmartServoFramework_VERSION_MAJOR 0)
set(SmartServoFramework_VERSION_MINOR 93)
set(SmartServoFramework_VERSION_PATCH 0)

set(ENABLE_DEBUG   1) # "Toggle debug traces"
set(ENABLE_COLORS  1) # "Toggle colored terminal output"

# Doxygen module
find_package(Doxygen) # FindDoxygen.cmake

if(DOXYGEN_FOUND)
    message(STATUS "* Doxygen found")
else(DOXYGEN_FOUND)
    message(WARNING " > Doxygen not found")
endif(DOXYGEN_FOUND)

# Detect build type. You can change it using "-DCMAKE_BUILD_TYPE="
if(CMAKE_BUILD_TYPE STREQUAL "Release")

    message(STATUS "* This is a RELEASE build. You can change it using -DCMAKE_BUILD_TYPE=Debug.")
    set(ENABLE_DEBUG 0)
    set(CMAKE_BUILD_TYPE release)

    # Custom cflags? Use "release mode" and "-O2" for better speed.
	if (UNIX)
		set(CMAKE_CXX_FLAGS "-O2 -Wall -std=c++11 -Wno-unused -Wno-unused-parameter")
	else()
		set(CMAKE_CXX_FLAGS "-O2 -Wall -std=c++11")
	endif()

else()

    message(STATUS "!! * This is a DEBUG build. You can change it using -DCMAKE_BUILD_TYPE=Release.")
    set(ENABLE_DEBUG 1)
    set(CMAKE_BUILD_TYPE debug)
    # Custom cflags? Use "debug mode" and "-O0 -g" if you want better support for GDB and Valgrind.
	if (UNIX)
		set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -std=c++11 -Wno-unused -Wno-unused-parameter")
	else()
		set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -std=c++11")
	endif()

endif(CMAKE_BUILD_TYPE STREQUAL "Release")

## Save options into a "cmake_defines.h" file, using "cmake_defines.h.in" template
#CONFIGURE_FILE(
#    "${CMAKE_SOURCE_DIR}/src/cmake_defines.h.in"
#    "${CMAKE_SOURCE_DIR}/src/cmake_defines.h"
#)

# Plateform specific
###############################################################################

# Linux specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    message(STATUS "Building on Linux plateform")
    set(EXTRALIBS "lockdev")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# Mac OS X specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    message(STATUS "Building on Mac OS X plateform")
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    message(STATUS "* Finding library IOKit: ${IOKIT_FRAMEWORK}")
    message(STATUS "* Finding library CoreFoundation: ${COREFOUNDATION_FRAMEWORK}")
    set(EXTRA_LIBS ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# Windows specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    message(STATUS "Building on Windows plateform")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

# Source files
###############################################################################

set(SmartServoFramework_SRC
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
add_library(SmartServoFramework_shared SHARED ${SmartServoFramework_SRC})
target_link_libraries(SmartServoFramework_shared ${EXTRA_LIBS})
set_target_properties(SmartServoFramework_shared PROPERTIES OUTPUT_NAME SmartServoFramework)

#if(CMAKE_BUILD_MODE STREQUAL "Static")
#    message(STATUS "** Static library will be built")
#    add_library(SmartServoFramework_static STATIC ${SmartServoFramework_SRC})
#    target_link_libraries(SmartServoFramework_static ${EXTRA_LIBS})
#    set_target_properties(SmartServoFramework_static PROPERTIES OUTPUT_NAME SmartServoFramework)
#endif(CMAKE_BUILD_MODE STREQUAL "Static")

# Install the shared library and its header into the system (optional step, requires root credentials)
# Relative to $<INSTALL_PREFIX>
###############################################################################

if(UNIX)
	install(TARGETS SmartServoFramework_shared DESTINATION lib/)
	install(DIRECTORY src/ DESTINATION include/SmartServoFramework FILES_MATCHING PATTERN "*.h")
else(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # TODO
endif()
