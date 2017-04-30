# SmartServoFramework "library" build system
# This file allows you to build a shared library of SmartServoFramework
################################################################################

cmake_minimum_required(VERSION 2.8)

project(SmartServoFramework)

set(SmartServoFramework_VERSION_MAJOR 0)
set(SmartServoFramework_VERSION_MINOR 96)
set(SmartServoFramework_VERSION_PATCH 0)

# Build settings
###############################################################################

set(ENABLE_DEBUG   1) # "Toggle debug traces"
set(ENABLE_COLORS  1) # "Toggle colored terminal output"

if(WIN32)
    # Turn on the ability to create folders to organize projects (.vcproj)
    # It creates "CMakePredefinedTargets" folder by default and adds CMake
    # defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # Set policy for interpretation of if() arguments as variables or keywords
    # when unquoted to NEW to avoid warnings.
    cmake_policy(SET CMP0054 NEW)

    if (CMAKE_VERSION VERSION_GREATER "3.3")
        include(GenerateExportHeader)
    endif()
endif()

# Build dependencies
################################################################################

# Doxygen OPTIONAL module
find_package(Doxygen) # FindDoxygen.cmake

if(DOXYGEN_FOUND)
    message(STATUS "* Doxygen found, you can use it to generate a nice API documentation!")
endif(DOXYGEN_FOUND)

# Build configuration
################################################################################

# Require C++11 standard by default
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add per-compiler custom rules
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message(STATUS "* Building with Clang compiler")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-function -Wno-unused-parameter -Wno-unused-variable")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    message(STATUS "* Building with Intel Cpp compiler")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    message(STATUS "* Building with GCC compiler")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message(STATUS "* Building with Visual Studio Cpp compiler")
endif()

# Detect build type (default is release)
# Change it using "-DCMAKE_BUILD_TYPE= Debug / Release / RelWithDebInfo / MinSizeRel"
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "!! * This is a DEBUG build. You can change it using -DCMAKE_BUILD_TYPE=Release.")
    set(ENABLE_DEBUG 1)
    # Custom cflags? Use "debug mode" and "-O0 -g" if you want better support for GDB and Valgrind.
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
else()
    message(STATUS "* This is a RELEASE build. You can change it using -DCMAKE_BUILD_TYPE=Debug.")
    set(ENABLE_DEBUG 0)
    # Custom cflags? Use "release mode" and "-O2" for better speed.
    set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

# Platform specific
################################################################################

# Linux specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    message(STATUS "Building on Linux plateform")
    #set(EXTRALIBS "lockdev")

endif()

# macOS specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    message(STATUS "Building on macOS plateform")
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    message(STATUS "* Finding library IOKit: ${IOKIT_FRAMEWORK}")
    message(STATUS "* Finding library CoreFoundation: ${COREFOUNDATION_FRAMEWORK}")
    set(EXTRA_LIBS ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})

endif()

# Windows specific stuff
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    message(STATUS "Building on Windows plateform")
    # Turn on the ability to create folders to organize projects (.vcproj)
    # It creates "CMakePredefinedTargets" folder by default and adds CMake
    # defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # Set policy for interpretation of if() arguments as variables or keywords
    # when unquoted to NEW to avoid warnings.
    cmake_policy(SET CMP0054 NEW)

    if(CMAKE_VERSION VERSION_GREATER "3.3")
        include(GenerateExportHeader)
    endif()

endif()

# Source files
###############################################################################

# Generate "SmartServoFramework_settings.h" file from a template
configure_file(
    "${CMAKE_SOURCE_DIR}/src/SmartServoFramework_settings.h.in"
    "${CMAKE_SOURCE_DIR}/src/SmartServoFramework_settings.h"
)

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
    src/SerialPortQt.cpp
    src/SerialPortQt.h
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
    src/ServoX.cpp
    src/ServoX.h
    src/Utils.cpp
    src/Utils.h
)

# Build
###############################################################################

# Build library
message(STATUS "** Building shared library")
add_library(SmartServoFramework_shared SHARED ${SmartServoFramework_SRC})
target_link_libraries(SmartServoFramework_shared ${EXTRA_LIBS})
set_target_properties(SmartServoFramework_shared PROPERTIES OUTPUT_NAME SmartServoFramework)

if(WIN32)
if (CMAKE_VERSION VERSION_GREATER "3.3")
    message("* Generating symbols export header")
    GENERATE_EXPORT_HEADER(SmartServoFramework_shared
                           BASE_NAME SmartServoFramework_shared
                           EXPORT_MACRO_NAME SmartServoFramework_EXPORT
                           EXPORT_FILE_NAME "${CMAKE_SOURCE_DIR}/src/SmartServoFramework_export.h"
                           STATIC_DEFINE SmartServoFramework_BUILT_AS_STATIC)
endif()
endif()

if(CMAKE_BUILD_MODE STREQUAL "Static")
    message(STATUS "** Building static library")
    add_library(SmartServoFramework_static STATIC ${SmartServoFramework_SRC})
    target_link_libraries(SmartServoFramework_static ${EXTRA_LIBS})
    set_target_properties(SmartServoFramework_static PROPERTIES OUTPUT_NAME SmartServoFramework)
endif(CMAKE_BUILD_MODE STREQUAL "Static")

# Install the shared library and its header into the system (optional step, requires root credentials)
# Relative to $<INSTALL_PREFIX>
###############################################################################

if(UNIX)
    install(TARGETS SmartServoFramework_shared DESTINATION lib/)
    install(DIRECTORY src/ DESTINATION include/SmartServoFramework/ FILES_MATCHING PATTERN "*.h")
elseif(WIN32)
    # TODO
endif()
