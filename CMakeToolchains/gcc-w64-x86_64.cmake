set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Compilers to use for C, C++
if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

if ("${ARM_GCC_PATH}" STREQUAL "")
    # Use default paths
    if(WIN32)
        set(CMAKE_C_COMPILER gcc.exe)
        set(CMAKE_CXX_COMPILER g++.exe)
    elseif(LINUX)
        set(CMAKE_C_COMPILER /usr/bin/gcc)
        set(CMAKE_CXX_COMPILER /usr/bin/g++) 
    endif()
endif()

# modify default behavior of FIND_XXX() commands
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
