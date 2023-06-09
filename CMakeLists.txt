cmake_minimum_required(VERSION 3.13)
set(PROJ_NAME soniclib)
project(SONICLIB C ASM)

set(CMAKE_C_STANDARD 99)

set(SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/invn/soniclib)
set(SENS_FW_PATH ${CMAKE_CURRENT_SOURCE_DIR}/invn/soniclib/sensor_fw)

message(STATUS "")
message(STATUS "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "COMPILER FLAGS")
message(STATUS "==============")
message(STATUS "CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}")
message(STATUS "CMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}")
message(STATUS "")
message(STATUS "CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")

set(INCLUDE_SHASTA_SUPPORT OFF CACHE BOOL
"Build soniclib for shasta (ICU) targets. Default is whitney targets \
(chx01).")

set(CHIRP_MAX_NUM_SENSORS 4 CACHE STRING "Max number of supported sensors.")
set(CHIRP_NUM_BUSES 2 CACHE STRING "Number of I2C/SPI buses used by sesnors.")
set(CHIRP_SENSOR_INT_PIN 1 CACHE STRING
    "Pin used for interrupts (shasta only).")
set(CHIRP_SENSOR_TRIG_PIN 1 CACHE STRING
    "Pin used for triggers (shasta only).")
set(CHIRP_INIT_FW_TYPE "FULL" CACHE STRING
"Type of init firmware (shasta only). NO_TX_OPTIMIZATION leaves out the \
transmit optimization feature but saves some space. NONE means no init \
firmware. Not all firmware variants support the NONE option.")
set_property(CACHE CHIRP_INIT_FW_TYPE PROPERTY STRINGS FULL
                                                       NO_TX_OPTIMIZATION
                                                       NONE)

set(INCLUDE_ALGO_EXTERNAL OFF CACHE BOOL
"If set, do not include gpt (rangefinder) support")

message(STATUS "CHIRP_INIT_FW_TYPE=${CHIRP_INIT_FW_TYPE}")

set(SOURCES ${SRC_PATH}/chbsp_dummy.c
            ${SRC_PATH}/ch_api.c
            ${SRC_PATH}/ch_common.c
            ${SRC_PATH}/ch_driver.c
            ${SRC_PATH}/ch_math_utils.c)

if(INCLUDE_SHASTA_SUPPORT)
    if (NOT INCLUDE_ALGO_EXTERNAL)
        set(SOURCES ${SOURCES}
                    ${SENS_FW_PATH}/icu_gpt/icu_gpt.c
                    ${SENS_FW_PATH}/icu_gpt/icu_gpt_fw.c)
    endif()
    if (CHIRP_INIT_FW_TYPE STREQUAL "FULL")
        set(SOURCES ${SOURCES}
                    ${SENS_FW_PATH}/icu_init/icu_init.c
                    ${SENS_FW_PATH}/icu_init/icu_init_fw.c)
    endif()
    if (CHIRP_INIT_FW_TYPE STREQUAL "NO_TX_OPTIMIZATION")
        set(SOURCES ${SOURCES}
                    ${SENS_FW_PATH}/icu_init-no-txopt/icu_init-no-txopt.c
                    ${SENS_FW_PATH}/icu_init-no-txopt/icu_init-no-txopt_fw.c)
    endif()
else()
    set(SOURCES ${SOURCES}
                ${SENS_FW_PATH}/ch101/ch101_gpr.c
                ${SENS_FW_PATH}/ch101/ch101_gpr_fw.c
                ${SENS_FW_PATH}/ch201/ch201_gprmt.c
                ${SENS_FW_PATH}/ch201/ch201_gprmt_fw.c
                ${SENS_FW_PATH}/chx01/chx01_freqsweep.c
                ${SENS_FW_PATH}/chx01/chx01_freqsweep_fw.c)
endif()


add_library(${PROJ_NAME} STATIC ${SOURCES})


target_include_directories(${PROJ_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(${PROJ_NAME}
    PRIVATE -DINVN_SONICLIB_INTERNAL_BOARD_CONFIG
            -DCHIRP_MAX_NUM_SENSORS=${CHIRP_MAX_NUM_SENSORS}
            -DCHIRP_NUM_BUSES=${CHIRP_NUM_BUSES})

if(INCLUDE_SHASTA_SUPPORT)
    target_compile_definitions(${PROJ_NAME} PUBLIC -DINCLUDE_SHASTA_SUPPORT
        PRIVATE -DCHIRP_SENSOR_INT_PIN=${CHIRP_SENSOR_INT_PIN}
                -DCHIRP_SENSOR_TRIG_PIN=${CHIRP_SENSOR_TRIG_PIN})
    if(${CHIRP_INIT_FW_TYPE} STREQUAL "NO_TX_OPTIMIZATION")
        target_compile_definitions(${PROJ_NAME} PRIVATE
            -DCHIRP_NO_TX_OPTIMIZATION)
    elseif(${CHIRP_INIT_FW_TYPE} STREQUAL "NONE")
        target_compile_definitions(${PROJ_NAME} PRIVATE -DCHIRP_NO_INIT_FW)
    endif()
    if (INCLUDE_ALGO_EXTERNAL)
        target_compile_definitions(${PROJ_NAME} PRIVATE -DINCLUDE_ALGO_EXTERNAL)
    endif()
else()
    target_compile_definitions(${PROJ_NAME} PRIVATE -DINCLUDE_WHITNEY_SUPPORT)
endif()

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
        DESTINATION ${CMAKE_SOURCE_DIR})
