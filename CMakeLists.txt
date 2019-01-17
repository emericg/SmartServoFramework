# SmartServoFramework "library" build system
# This file allows you to build a shared library of SmartServoFramework
################################################################################

project(SmartServoFramework)

set(SmartServoFramework_VERSION_MAJOR 0)
set(SmartServoFramework_VERSION_MINOR 99)
set(SmartServoFramework_VERSION_PATCH 0)

cmake_minimum_required(VERSION 3.0)

# Build settings
################################################################################

set(ENABLE_DEBUG   1) # "Toggle debug traces"
set(ENABLE_COLORS  1) # "Toggle colored terminal output"

# Build dependencies
################################################################################

# Doxygen OPTIONAL module
find_package(Doxygen) # FindDoxygen.cmake

if(DOXYGEN_FOUND)
    message(STATUS "* Doxygen found, you can use it to generate a nice API documentation!")
endif(DOXYGEN_FOUND)

# Build configuration
################################################################################

option(BUILD_STATIC_LIBS "Build static library" ON)
option(BUILD_SHARED_LIBS "Build shared library" ON)

# Require C++11 standard by default
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

    if(CMAKE_VERSION VERSION_GREATER "3.4")
       set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)
   endif()

endif()

# Source files
################################################################################

# Generate "SmartServoFramework_settings.h" file from a template
configure_file(
    "${CMAKE_SOURCE_DIR}/SmartServoFramework/SmartServoFramework_settings.h.in"
    "${CMAKE_SOURCE_DIR}/SmartServoFramework/SmartServoFramework_settings.h"
)

set(SmartServoFramework_SRC
    SmartServoFramework/minitraces.cpp
    SmartServoFramework/minitraces_conf.h
    SmartServoFramework/minitraces.h
    SmartServoFramework/SimpleAPI.h
    SmartServoFramework/ManagedAPI.h
    SmartServoFramework/ControlTables.cpp
    SmartServoFramework/ControlTables.h
    SmartServoFramework/ControlTablesDynamixel.h
    SmartServoFramework/ControlTablesHerkuleX.h
    SmartServoFramework/Dynamixel.cpp
    SmartServoFramework/Dynamixel.h
    SmartServoFramework/DynamixelTools.cpp
    SmartServoFramework/DynamixelTools.h
    SmartServoFramework/DynamixelSimpleAPI.cpp
    SmartServoFramework/DynamixelSimpleAPI.h
    SmartServoFramework/DynamixelController.cpp
    SmartServoFramework/DynamixelController.h
    SmartServoFramework/HerkuleX.cpp
    SmartServoFramework/HerkuleX.h
    SmartServoFramework/HerkuleXTools.cpp
    SmartServoFramework/HerkuleXTools.h
    SmartServoFramework/HerkuleXSimpleAPI.cpp
    SmartServoFramework/HerkuleXSimpleAPI.h
    SmartServoFramework/HerkuleXController.cpp
    SmartServoFramework/HerkuleXController.h
    SmartServoFramework/SerialPort.cpp
    SmartServoFramework/SerialPort.h
    SmartServoFramework/SerialPortQt.cpp
    SmartServoFramework/SerialPortQt.h
    SmartServoFramework/SerialPortLinux.cpp
    SmartServoFramework/SerialPortLinux.h
    SmartServoFramework/SerialPortMacOS.cpp
    SmartServoFramework/SerialPortMacOS.h
    SmartServoFramework/SerialPortWindows.cpp
    SmartServoFramework/SerialPortWindows.h
    SmartServoFramework/ServoTools.cpp
    SmartServoFramework/ServoTools.h
    SmartServoFramework/ServoController.cpp
    SmartServoFramework/ServoController.h
    SmartServoFramework/Servo.cpp
    SmartServoFramework/Servo.h
    SmartServoFramework/ServoDynamixel.cpp
    SmartServoFramework/ServoDynamixel.h
    SmartServoFramework/ServoAX.cpp
    SmartServoFramework/ServoAX.h
    SmartServoFramework/ServoEX.cpp
    SmartServoFramework/ServoEX.h
    SmartServoFramework/ServoMX.cpp
    SmartServoFramework/ServoMX.h
    SmartServoFramework/ServoXL.cpp
    SmartServoFramework/ServoXL.h
    SmartServoFramework/ServoX.cpp
    SmartServoFramework/ServoX.h
    SmartServoFramework/ServoHerkuleX.cpp
    SmartServoFramework/ServoHerkuleX.h
    SmartServoFramework/ServoDRS.cpp
    SmartServoFramework/ServoDRS.h
)

set(SmartServoFramework_PUBLIC_HEADERS
    SmartServoFramework/SimpleAPI.h
    SmartServoFramework/ManagedAPI.h
    SmartServoFramework/ControlTablesDynamixel.h
    SmartServoFramework/ControlTables.h
    SmartServoFramework/ControlTablesHerkuleX.h
    SmartServoFramework/Dynamixel.h
    SmartServoFramework/DynamixelTools.h
    SmartServoFramework/DynamixelSimpleAPI.h
    SmartServoFramework/DynamixelController.h
    SmartServoFramework/HerkuleX.h
    SmartServoFramework/HerkuleXSimpleAPI.h
    SmartServoFramework/HerkuleXController.h
    SmartServoFramework/HerkuleXTools.h
    SmartServoFramework/SerialPort.h
    SmartServoFramework/SerialPortLinux.h
    SmartServoFramework/SerialPortMacOS.h
    SmartServoFramework/SerialPortWindows.h
    SmartServoFramework/ServoTools.h
    SmartServoFramework/ServoController.h
    SmartServoFramework/Servo.h
    SmartServoFramework/ServoDynamixel.h
    SmartServoFramework/ServoAX.h
    SmartServoFramework/ServoEX.h
    SmartServoFramework/ServoMX.h
    SmartServoFramework/ServoXL.h
    SmartServoFramework/ServoX.h
    SmartServoFramework/ServoHerkuleX.h
    SmartServoFramework/ServoDRS.h
)

# Build library (static and shared rules)
################################################################################

# Build library
if (BUILD_SHARED_LIBS)
    message(STATUS "** Building shared library")
    add_library(SmartServoFramework_shared SHARED ${SmartServoFramework_SRC})
    target_link_libraries(SmartServoFramework_shared ${EXTRA_LIBS})
    set_target_properties(SmartServoFramework_shared PROPERTIES OUTPUT_NAME smartservoframework)
endif (BUILD_SHARED_LIBS)

if (BUILD_STATIC_LIBS)
    message(STATUS "** Building static library")
    add_library(SmartServoFramework_static STATIC ${SmartServoFramework_SRC})
    target_link_libraries(SmartServoFramework_static ${EXTRA_LIBS})
    set_target_properties(SmartServoFramework_static PROPERTIES OUTPUT_NAME smartservoframework)
endif (BUILD_STATIC_LIBS)

# Deployment
################################################################################

# pkg-config integration
set(PROJECT_VERSION "${SmartServoFramework_VERSION_MAJOR}.${SmartServoFramework_VERSION_MINOR}")
set(LIB_SUFFIX "" CACHE STRING "Define suffix of directory name")
set(EXEC_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE PATH "Installation prefix for executables and object code libraries" FORCE)
set(BIN_INSTALL_DIR ${EXEC_INSTALL_PREFIX}/bin CACHE PATH "Installation prefix for user executables" FORCE)
set(LIB_INSTALL_DIR ${EXEC_INSTALL_PREFIX}/lib${LIB_SUFFIX} CACHE PATH "Installation prefix for object code libraries" FORCE)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/SmartServoFramework CACHE PATH "Installation prefix for header files" FORCE)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libsmartservoframework.pc.cmake ${CMAKE_CURRENT_BINARY_DIR}/libsmartservoframework.pc)

# Install the libraries and its header into the system (optional step, requires root credentials)
# Relative to $<CMAKE_INSTALL_PREFIX>
if (UNIX)
    if (BUILD_SHARED_LIBS)
        install(TARGETS SmartServoFramework_shared DESTINATION lib/)
    endif (BUILD_SHARED_LIBS)
    if (BUILD_STATIC_LIBS)
        install(TARGETS SmartServoFramework_static DESTINATION lib/)
    endif (BUILD_STATIC_LIBS)

    install(FILES ${SmartServoFramework_PUBLIC_HEADERS} DESTINATION include/SmartServoFramework/)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libsmartservoframework.pc DESTINATION lib/pkgconfig/)
endif(UNIX)
