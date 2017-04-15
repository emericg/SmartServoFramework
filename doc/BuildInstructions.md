Build instructions
------------------

### Dependencies

You will need a modern C++11 capable compiler:
* GCC >= 4.8
* LLVM >= 3.6
* MSVC >= 2015

Build system:
* CMake (**ONLY** needed to build the standalone library)
* Scons (**ONLY** needed to build the examples)
* Doxygen (**ONLY** needed to generate the documentation)

Using Linux?
* liblockdev (**OPTIONAL**) (used to lock the serial port to a single software instance, which will save you from a lot of potential head scratching errors...)

### Building SmartServoFramework library (Linux and macOS)

> $ cd SmartServoFramework/build/
> $ cmake ..
> $ make

The shared library will be located inside the `build/` directory.

### Building test softwares (Linux and macOS)

Various test softwares are available in the `examples/` directory for you to play with:

* ex_simple: Control up to four servos with your keyboard using the 'Simple API'.
* ex_simple_threaded: ex_simple variant, with servos actions and keyboard handled in two separate threads.
* ex_controller: Control four servos with your keyboard using the 'Managed API'.
* ex_sinus_control: Control a servo with sinusoid curve for both speed and position. Enable OpenCV to get a nice position/speed graph.
* ex_advance_scanner: Scan serial ports for Dynamixel servos, for all IDs and all (but configurable) serial port speeds.

You can build them all at once:
> $ cd SmartServoFramework/examples/
> $ scons

The executables will be located inside the `examples/build/` directory.

### Building SmartServoFramework library (Windows)

TODO

### Building test softwares (Windows)

TODO
