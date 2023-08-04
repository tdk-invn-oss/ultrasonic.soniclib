/*! \file icu_gpt.h
 *
 * \brief Internal definitions for the Chirp ICU General Purpose Transceiver (GPT) firmware.
 *
 * This file contains various definitions and values for use with the icu_gpt
 * sensor firmware.
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

#ifndef ICU_GPT_H_
#define ICU_GPT_H_

#include <invn/soniclib/details/icu.h>
#include <invn/soniclib/soniclib.h>
#include <stdint.h>

#define ICU_GPT_MAX_SAMPLES (IQ_SAMPLES_MAX)  // from shasta_external_regs.h

extern const char *icu_gpt_version;  // version string in fw .c file
extern const uint8_t icu_gpt_fw_text[];
extern const uint8_t icu_gpt_fw_vec[];
extern const uint16_t icu_gpt_text_size;
extern const uint16_t icu_gpt_vec_size;

uint16_t get_icu_gpt_fw_ram_init_addr(void);
uint16_t get_icu_gpt_fw_ram_init_size(void);

const unsigned char *get_ram_icu_gpt_init_ptr(void);

uint8_t icu_gpt_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t dev_num, uint8_t bus_index);

#endif /* ICU_GPT_H_ */
