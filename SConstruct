# SmartServoFramwork build system
# Emeric Grange <emeric.grange@inria.fr>

# http://www.scons.org/
# http://www.scons.org/wiki/IDEIntegration

import sys
import subprocess
from distutils.version import StrictVersion


# Initialize the environment
###############################################################################

if ARGUMENTS.get('debug', 0):
    print "> DEBUG build"
    env = Environment(CCFLAGS = ' -g -Wno-unused-parameter ')
else:
    env = Environment(CCFLAGS = ' -O2 -Wno-unused-parameter ')


# Multiplatform build
###############################################################################

if sys.platform.startswith('linux') == True:

    print '> LINUX platform'
    CC_VERSION = subprocess.check_output([env['CC'], "-dumpversion"]).strip('\n')
    if env['CC'] == 'gcc':
        print '> Gcc ' + CC_VERSION + ' compiler'
        if StrictVersion(CC_VERSION) < StrictVersion('4.6'):
            print 'You need at least GCC 4.6+ to use C++11 features!'
            exit(0)
        elif StrictVersion(CC_VERSION) < StrictVersion('4.8'):
            # GCC 4.6 needs a different flag to enable C++11
            env.Append(CCFLAGS = "-std=c++0x -pthread ")
        else:
            env.Append(CCFLAGS = "-std=c++11 -pthread ")
    elif env['CC'] == 'clang':
        print '> LLVM ' + CC_VERSION + ' compiler'
        env.Append(CCFLAGS = "-std=c++11 ")
    else:
        print '> WARNING > Unknown compiler > ' + env['CC']
        exit(0)

    # Additional libraries
    libraries = ['pthread', 'lockdev']
    libraries_paths = ['']

elif sys.platform.startswith('win') == True:

    print '> WINDOWS platform'
    if env['CC'] == 'MVS':
        print '> Visual Studio compiler'
        env.Append(CCFLAGS = "-std=c++11 ")
        env.Append(tools = "msvc")
    elif env['CC'] == 'gcc':
        print '> MinGW compiler'
        env.Append(CCFLAGS = "-std=c++11 -pthread -winpthreads ")
    else:
        print '> WARNING > Unknown compiler > ' + env['CC']

    # Additional libraries
    libraries = ['']
    libraries_paths = ['']

elif sys.platform == 'darwin':

    print '> MAC OS platform'
    if env['CC'] == 'gcc':
        print '> Gcc compiler'
        env.Append(CCFLAGS = "-std=c++11 -stdlib=libstdc++ -pthread ")
    elif env['CC'] == 'clang':
        print '> LLVM compiler'
        env.Append(CCFLAGS = "-std=c++11 -stdlib=libc++ ")
    else:
        print '> WARNING > Unknown compiler > ' + env['CC']

    # Additional libraries
    libraries = ['IOKit']
    libraries_paths = ['']

else:
    print '> WARNING > Unknown operating system > ' + sys.platform


# Source files
###############################################################################

src_framework = [env.Object("src/SerialPort.cpp"), env.Object("src/SerialPortLinux.cpp"), env.Object("src/SerialPortMacOS.cpp"), env.Object("src/SerialPortWindows.cpp"),
                 env.Object("src/minitraces.cpp"), env.Object("src/ControlTables.cpp"), env.Object("src/Utils.cpp"), env.Object("src/ControllerAPI.cpp"),env.Object("src/Servo.cpp"),
                 env.Object("src/Dynamixel.cpp"), env.Object("src/DynamixelTools.cpp"), env.Object("src/DynamixelSimpleAPI.cpp"), env.Object("src/DynamixelController.cpp"),
                 env.Object("src/ServoDynamixel.cpp"), env.Object("src/ServoAX.cpp"), env.Object("src/ServoEX.cpp"), env.Object("src/ServoMX.cpp"), env.Object("src/ServoXL.cpp"),
                 env.Object("src/HerkuleX.cpp"), env.Object("src/HerkuleXTools.cpp"), env.Object("src/HerkuleXSimpleAPI.cpp"), env.Object("src/HerkuleXController.cpp"),
                 env.Object("src/ServoHerkuleX.cpp"), env.Object("src/ServoDRS.cpp")]


# Build examples
###############################################################################

env.Program(target = 'build/ex_basic_test', source = ["examples/ex_basic_test.cpp", src_framework], LIBS = libraries, LIBPATH = libraries_paths)
env.Program(target = 'build/ex_simple', source = ["examples/ex_simple.cpp"] + src_framework, LIBS = libraries, LIBPATH = libraries_paths)
env.Program(target = 'build/ex_simple_threaded', source = ["examples/ex_simple_threaded.cpp"] + src_framework, LIBS = libraries, LIBPATH = libraries_paths)
env.Program(target = 'build/ex_controller', source = ["examples/ex_controller.cpp"] + src_framework, LIBS = libraries, LIBPATH = libraries_paths)
env.Program(target = 'build/ex_sinus_control', source = ["examples/ex_sinus_control.cpp"] + src_framework, LIBS = libraries + ["opencv_core", "opencv_highgui"], LIBPATH = libraries_paths)
env.Program(target = 'build/ex_advance_scanner', source = ["examples/ex_advance_scanner.cpp"] + src_framework, LIBS = libraries, LIBPATH = libraries_paths)
