/*! \file icu_shasta_algo_structs.h
 *
 * \brief Local SonicLib version of sensor algorithm header file
 *
 * This is simply a wrapper header file that uses build symbols to select the
 * appropriate version of the icu_shasta_algo_structs.h file located in a
 * specific subdirectory under sensor_fw (which must be in the include path).
 */

/*
 Copyright 2016-2023, InvenSense, Inc.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED.

 */

#ifndef SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H
#define SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <invn/soniclib/soniclib.h>

#if defined(INCLUDE_ALGO_RANGEFINDER)
#include <invn/soniclib/sensor_fw/icu_gpt/icu_shasta_algo_structs.h>

/* add new sensor f/w interfaces here */

#elif defined(INCLUDE_ALGO_NONE)
#include <invn/soniclib/sensor_fw/icu_init/icu_shasta_algo_structs.h>  // no algorithm - initialization only
#endif

#ifdef __cplusplus
}
#endif

#endif  // SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H
