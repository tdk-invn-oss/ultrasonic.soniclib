/*! \file ch201_gprmt.h
 *
 * \brief Internal definitions for the Chirp CH201 GPR Multi-threshold sensor firmware.
 *
 * This file contains register offsets and other values for use with the CH201 GPR 
 * Multi-threshold sensor firmware.  These values are subject to change without notice.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 Copyright 2016-2023, InvenSense, Inc.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED.

 */

#ifndef CH201_GPRMT_H_
#define CH201_GPRMT_H_

#include "ch201.h"
#include <invn/soniclib/soniclib.h>
#include <stdint.h>

/* GPR with multi thresholds firmware registers */
#define CH201_GPRMT_REG_OPMODE 			0x01
#define CH201_GPRMT_REG_TICK_INTERVAL 	0x02
#define CH201_GPRMT_REG_LOW_GAIN_RXLEN	0x04
#define CH201_GPRMT_REG_PERIOD 			0x05
#define CH201_GPRMT_REG_CAL_TRIG 		0x06
#define CH201_GPRMT_REG_MAX_RANGE 		0x07
#define CH201_GPRMT_REG_THRESH_LEN_0 	0x08
#define CH201_GPRMT_REG_THRESH_LEN_1 	0x09
#define CH201_GPRMT_REG_CAL_RESULT 		0x0A
#define CH201_GPRMT_REG_THRESH_LEN_2 	0x0C
#define CH201_GPRMT_REG_THRESH_LEN_3 	0x0D
#define CH201_GPRMT_REG_TX_LENGTH     	0x10
#define CH201_GPRMT_REG_ST_RANGE 		0x12
#define CH201_GPRMT_REG_READY 			0x14
#define CH201_GPRMT_REG_THRESH_LEN_4 	0x15
#define CH201_GPRMT_REG_THRESHOLDS		0x16	// start of array of six 2-byte threshold levels
#define CH201_GPRMT_REG_TOF_SF 			0x22
#define CH201_GPRMT_REG_TOF 			0x24
#define CH201_GPRMT_REG_AMPLITUDE 		0x26
#define CH201_GPRMT_REG_DATA 			0x28

#define CH201_GPRMT_MAX_SAMPLES			(450)	// max number of samples
#define CH201_GPRMT_NUM_THRESHOLDS		(6)		// total number of thresholds

extern const char *ch201_gprmt_version;		// version string in fw .c file
extern const uint8_t ch201_gprmt_fw_text[];
extern const uint8_t ch201_gprmt_fw_vec[];
extern const uint16_t ch201_gprmt_text_size;
extern const uint16_t ch201_gprmt_vec_size;

uint16_t get_ch201_gprmt_fw_ram_init_addr(void);
uint16_t get_ch201_gprmt_fw_ram_init_size(void);

const unsigned char * get_ram_ch201_gprmt_init_ptr(void);

uint8_t ch201_gprmt_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t dev_num, uint8_t bus_index);


#endif
