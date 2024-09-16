# SonicLib

## Summary

### The main API documentation is available [here](https://tdk-invn-oss.github.io/ultrasonic.soniclib)

SonicLib is a set of API functions and sensor driver routines designed so
an embedded C application can easily control TDK/Invensense ultrasonic sensors (originally
developed by Chirp Microsystems).  SonicLib allows an application developer to manage and obtain 
ultrasonic data from one or more devices, without needing to develop special low-level code 
to interact with the sensors directly.

The SonicLib API functions provide a consistent interface for an application to use
different sensors in various situations.  This is especially important, because
all TDK ultrasonic sensors are completely software-defined, including the register map.  The
SonicLib interfaces allow an application to use different sensor firmware images,
without requiring code changes.

\note All operation of the sensor is controlled through the set of functions, data structures,
and symbolic values defined in the soniclib.h header file.  You should not need to modify this file 
or the SonicLib functions, or use lower-level internal functions such as described in 
the ch_driver.h file.  Using any of these non-public methods will reduce your ability to 
benefit from future enhancements and releases from Chirp.

## SonicLib API
### The main documentation for the SonicLib API is in the soniclib.h header file.
That file contains definitions of all public interfaces that applications can
use to interact with the sensor.

There are over 100 different functions available in the SonicLib API, although a typical 
application will only use a small subset of them.  There are many \a ch_set_XXX() and
\a ch_get_XXX() functions that set and return specific values for the device, so it should 
never be necessary to directly access fields in the device descriptor or other internal
structures.

## Required Board Support Package (BSP)
### The main documentation for the BSP interface is in the chirp_bsp.h header file.
SonicLib also defines interfaces for a set of board support package (BSP) 
functions that are specific to the hardware platform being used.  These are functions
that are not part of SonicLib, but that SonicLib will call when necessary
to interact with the peripheral devices and other resources on the board. 
These include GPIO pins, timers, enabling/disabling interrupts, etc.

The chirp_bsp.h file contains information on implementing these functions as well as
the interface definitions and which functions are required, recommended, or optional 
for your situation.

The BSP implementation in a final product is ultimately the responsibility of
the development team. However, example BSP's are available for certain 
evaluation boards for use during development and as a template for custom
designs.  Typically, BSPs that use the same MCU family will share common calls
to the MCU vendor's I/O library, varying mostly in pin assignments, etc.

Contact TDK/InvenSense for information on BSP availability.

## Chirp Driver (internal)
SonicLib includes internal driver functions to interact directly with the sensor.
These modules are distributed as source code to simplify integration with your embedded
application and are supplied for reference only.  These functions are described in the 
ch_driver.h header file.  You should not modify these functions or call them directly 
from your application.

Similarly, the `details` subdirectory within the SonicLib distribution contains
header files and other internal definitions that are needed to build SonicLib
with your application.  However, these files and their contents are subject
to change without notice and should not be referenced directly.

## CMake Support
A `CMakeLists.txt` is provided for easy integration of SonicLib into a CMake-based
project. Generally, all that is required is to use the `add_subdirectory`
command to add the directory containing this `CMakeLists.txt` to your project.
You then get a target called `soniclib` that you can use with the target-based
CMake commands (`target_link_libraries`, `target_include_directories`).

Additionally, there are several CMake variables that control various build
options. These are as follows. See the
`invn/soniclib/details/chirp_board_config.h` header for details.

* `CHIRP_MAX_NUM_SENSORS`: Maximum number of sensors supported.
* `CHIRP_NUM_BUSES`: Number of SPI/I2C buses supported.
* `CHIRP_SENSOR_INT_PIN`: (ICU-x0201 only) Pin to use for interrupts.
* `CHIRP_SENSOR_TRIG_PIN`: (ICU-x0201 only) Pin to use for triggers.
* `CHIRP_MAIN_FW_TYPE` : Select the main firmware to build with, can be one of `NONE` (fw built externally) or one of the firmware embedded by default with Soniclib `GPT` (for shasta devices), `CH101_GPR`, `CH201_GPRMT` or `CHX01_FREQSWEEP`
* `CHIRP_INIT_FW_TYPE`: Select an init firmware to build, if necessary in one of `FULL`, `NO_TX_OPTIMIZATION`, or `NONE`.
* `CHIRP_LOG_LEVEL` : Define the log level to use for the Soniclib internal log messages

These variables can be set through the CMake GUI, through command line -D
options in the main CMake command, or manually through the `CMakeCache.txt` file
post-configuration.

For example, the following cmake command configures the SonicLib build for
shasta support with the `NO_TX_OPTIMIZATION` init firmware. Below we are using a
toolchain file for an STM32 target. You can replace this with the toolchain file
needed by your target. Note the `` ` `` character is for powershell
line-continuation.

```powershell
cmake -G "MinGW Makefiles" `
    -DCMAKE_TOOLCHAIN_FILE="CMakeToolchains/arm-none-eabi-m4.cmake" `
    -DCHIRP_MAIN_FW_TYPE:STRING=GPT `
    -DCHIRP_INIT_FW_TYPE:STRING=NO_TX_OPTIMIZATION `
    -B build
```

To build Soniclib from a parent cmake project, you can follow this example :

```cmake
set(CHIRP_MAX_NUM_SENSORS 1 CACHE STRING "one sensor connected" FORCE)
set(CHIRP_NUM_BUSES 1 CACHE STRING "one sensor connected" FORCE)
set(INCLUDE_SHASTA_SUPPORT ON CACHE BOOL "using ICU sensors" FORCE)
set(CHIRP_SENSOR_INT_PIN 1 CACHE STRING "use INT1 in freerun mode" FORCE)
set(CHIRP_SENSOR_TRIG_PIN 1 CACHE STRING "use INT1 in freerun mode" FORCE)
set(CHIRP_INIT_FW_TYPE "NO_TX_OPTIMIZATION" CACHE STRING "using fw presencev2" FORCE)
set(CHIRP_LOG_LEVEL 2 CACHE STRING "log=info" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/invn/soniclib/)

...

target_link_libraries(app PRIVATE soniclib)
```

## Copyright
&copy; Copyright 2016-2024, TDK/InvenSense.  All rights reserved.
