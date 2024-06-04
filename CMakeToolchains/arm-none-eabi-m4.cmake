cmake_minimum_required(VERSION 3.12)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# Needed to make the test compile pass
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY" CACHE STRING "" FORCE)

file(TO_CMAKE_PATH "$ENV{ARM_GCC_PATH}" ARM_GCC_PATH)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(ARCH_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")

if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

if ("${ARM_GCC_PATH}" STREQUAL "")
    # Use default paths
    if(WIN32)
        set(CMAKE_C_COMPILER arm-none-eabi-gcc.exe)
        set(CMAKE_CXX_COMPILER arm-none-eabi-g++.exe)
    elseif(LINUX)
        set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
        set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++) 
    endif()
else()
    if(WIN32)
        set(CMAKE_C_COMPILER "${ARM_GCC_PATH}/bin/arm-none-eabi-gcc.exe")
        set(CMAKE_CXX_COMPILER "${ARM_GCC_PATH}/bin/arm-none-eabi-g++.exe")
    else()
        set(CMAKE_C_COMPILER "${ARM_GCC_PATH}/bin/arm-none-eabi-gcc")
        set(CMAKE_CXX_COMPILER "${ARM_GCC_PATH}/bin/arm-none-eabi-g++")
    endif()
endif()

# Compile flags
set(CMAKE_C_FLAGS 
"${ARCH_FLAGS} -fno-strict-aliasing -ffunction-sections -fdata-sections \
-fno-exceptions -lgcc -lc -lm -lrdimon -Wall -Wextra -pedantic -Werror")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS}")

set(CMAKE_C_FLAGS_DEBUG "-Og -g -fno-omit-frame-pointer -funwind-tables -mno-sched-prolog")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

# Link flags for code size
set(GC "-Wl,--gc-sections")
set(MAP "-Wl,-Map=output.map")
# set(NOSYS "--specs=nosys.specs")
set(RDIMON "--specs=rdimon.specs")
# set(USE_NANO "--specs=nano.specs")

set(LDFLAGS "${RDIMON} ${GC} ${MAP} -lc")
set(CMAKE_EXE_LINKER_FLAGS "${LDFLAGS}")
