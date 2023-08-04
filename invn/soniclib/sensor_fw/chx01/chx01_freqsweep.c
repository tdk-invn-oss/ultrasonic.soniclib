/*!
 * \file chx01_freqsweep.c
 *
 * \brief Chirp CHx01 Frequency Sweep firmware interface
 *
 * This file contains function definitions to interface a specific sensor firmware
 * package to SonicLib, including the main initialization routine for the firmware.
 * That routine initializes various fields within the \a ch_dev_t device descriptor
 * and specifies the proper functions to implement SonicLib API calls.  Those may
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright 2016-2023, InvenSense, Inc.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED.

 */

#include <invn/soniclib/soniclib.h>
#include <invn/soniclib/details/ch_common.h>
#include <invn/soniclib/details/ch_math_utils.h>
#include "chx01_freqsweep.h"

extern const uint8_t chx01_freqsweep_fw[CHX01_FW_SIZE];

uint32_t chx01_freqsweep_get_range(ch_dev_t *dev_ptr, ch_range_t range_type);

uint8_t chx01_freqsweep_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index,
                             uint8_t bus_index) {

	(void)grp_ptr;

	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index        = io_index;
	dev_ptr->bus_index       = bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text              = chx01_freqsweep_fw_text;
	dev_ptr->fw_text_size         = chx01_freqsweep_text_size;
	dev_ptr->fw_vec               = chx01_freqsweep_fw_vec;
	dev_ptr->fw_vec_size          = chx01_freqsweep_vec_size;
	dev_ptr->fw_version_string    = chx01_freqsweep_version;
	dev_ptr->ram_init             = get_ram_chx01_freqsweep_init_ptr();
	dev_ptr->get_fw_ram_init_size = get_chx01_freqsweep_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr = get_chx01_freqsweep_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer = ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result     = chx01_freqsweep_store_pt_result;
	dev_ptr->store_op_freq       = chx01_freqsweep_store_op_freq;
	dev_ptr->store_bandwidth     = ch_common_store_bandwidth;
	dev_ptr->store_scalefactor   = chx01_freqsweep_store_scale_factor;
	dev_ptr->get_locked_state    = chx01_freqsweep_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load             = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode            = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples     = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range       = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range    = NULL;
	dev_ptr->api_funcs.set_rx_holdoff      = ch_common_set_rx_holdoff;
	dev_ptr->api_funcs.get_rx_holdoff      = ch_common_get_rx_holdoff;
	dev_ptr->api_funcs.get_range           = (ch_get_range_func_t)chx01_freqsweep_get_range;
	dev_ptr->api_funcs.get_amplitude       = chx01_freqsweep_get_amplitude;
	dev_ptr->api_funcs.get_iq_data         = (ch_get_iq_data_func_t)chx01_freqsweep_get_iq_data;
	dev_ptr->api_funcs.samples_to_mm       = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples       = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds      = NULL;  // not supported
	dev_ptr->api_funcs.get_thresholds      = NULL;  // not supported

	/* Init max sample count */
	dev_ptr->max_samples = CHX01_FREQSWEEP_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	return 0;
}

uint8_t chx01_freqsweep_set_rxqueue_item(ch_dev_t *dev_ptr, uint8_t queue_index, uint8_t samples, uint8_t attenuation,
                                         uint8_t gain) {
	uint8_t ret = !dev_ptr || queue_index >= CHX01_FREQSWEEP_RXQUEUE_ITEMS ||
	              samples >= 1 << CHX01_FREQSWEEP_RXQUEUE_BITS_SAMPLES ||
	              attenuation >= 1 << CHX01_FREQSWEEP_RXQUEUE_BITS_ATTEN ||
	              gain >= 1 << CHX01_FREQSWEEP_RXQUEUE_BITS_GAIN;

	if (!ret) {
		uint16_t item = (uint16_t)samples << CHX01_FREQSWEEP_RXQUEUE_BITPOS_SAMPLES |
		                (uint16_t)attenuation << CHX01_FREQSWEEP_RXQUEUE_BITPOS_ATTEN |
		                (uint16_t)gain << CHX01_FREQSWEEP_RXQUEUE_BITPOS_GAIN;

		ret = chdrv_write_word(dev_ptr, CHX01_FREQSWEEP_REG_RXQUEUE + queue_index * sizeof(item), item);
	}

	return ret;
}

uint8_t chx01_freqsweep_set_threshold(ch_dev_t *dev_ptr, uint16_t threshold) {
	uint8_t ret = 1;
	if (dev_ptr->sensor_connected) {
		ret = chdrv_write_word(dev_ptr, CHX01_FREQSWEEP_REG_THRESHOLD, threshold);
	}
	return ret;
}

uint8_t chx01_freqsweep_set_holdoff(ch_dev_t *dev_ptr, uint8_t holdoff) {
	uint8_t ret = !dev_ptr || !dev_ptr->sensor_connected || !holdoff;
	if (!ret)
		ret = chdrv_write_byte(dev_ptr, CHX01_FREQSWEEP_REG_HOLDOFF, holdoff);
	return ret;
}

uint32_t chx01_freqsweep_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint8_t tof_reg;
	uint32_t range = CH_NO_TARGET;
	uint16_t time_of_flight;
	uint32_t tof_scale_factor;
	int err;

	if (dev_ptr->sensor_connected) {

		tof_reg = CHX01_FREQSWEEP_REG_TOF;

		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight);

		if (!err && (time_of_flight != UINT16_MAX)) {  // If object detected

			if (dev_ptr->tof_scale_factor == 0) {
				chx01_freqsweep_store_scale_factor(dev_ptr);
			}
			tof_scale_factor = dev_ptr->tof_scale_factor;

			if (tof_scale_factor != 0) {
				uint32_t num = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->rtc_cal_pulse_ms * (uint32_t)time_of_flight);
				uint32_t den = ((uint32_t)dev_ptr->rtc_cal_result * tof_scale_factor) >> 12;  // TODO need define

				range = (num / den);

				if (range_type == CH_RANGE_ECHO_ONE_WAY) {
					range /= 2;
				}

				/* Adjust for oversampling, if used */
				range >>= dev_ptr->oversample;
			}
		}
	}
	return range;
}

uint16_t chx01_freqsweep_get_amplitude(ch_dev_t *dev_ptr) {
	uint16_t intensity = 0xFFFF;
	chdrv_read_word(dev_ptr, CHX01_FREQSWEEP_REG_AMPLITUDE, &intensity);
	return intensity;
}

uint8_t chx01_freqsweep_get_iq_data(ch_dev_t *dev_ptr, uint8_t /*ch_iq_sample_t*/ *buf_ptr, uint16_t start_sample,
                                    uint16_t num_samples, uint8_t /*ch_io_mode_t*/ mode) {

	uint16_t iq_data_addr = CHX01_FREQSWEEP_REG_DATA;
	uint16_t num_bytes    = (num_samples * sizeof(ch_iq_sample_t));
	uint8_t error         = 0;

	iq_data_addr += (start_sample * sizeof(ch_iq_sample_t));

	if (iq_data_addr > UINT8_MAX) {
		error = 1;
	}

	if (mode != CH_IO_MODE_BLOCK) {
		error = 1;
	}

	if (!error) {
		error = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *)buf_ptr, num_bytes);
	}

	return error;
}

uint8_t chx01_freqsweep_get_locked_state(ch_dev_t *dev_ptr) {
	uint8_t locked = CHX01_FREQSWEEP_READY_NOTLOCKED;
	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, CHX01_FREQSWEEP_REG_READY, &locked);
	}
	return (locked == CH101_COMMON_READY_FREQ_LOCKED);
}

void chx01_freqsweep_store_pt_result(ch_dev_t *dev_ptr) {
	chdrv_read_word(dev_ptr, CHX01_FREQSWEEP_REG_CAL_RESULT, &(dev_ptr->rtc_cal_result));

	dev_ptr->rtc_frequency = (dev_ptr->rtc_cal_result * 1000) / dev_ptr->group->rtc_cal_pulse_ms;
}

void chx01_freqsweep_store_op_freq(ch_dev_t *dev_ptr) {
	dev_ptr->op_frequency = chx01_freqsweep_get_op_freq(dev_ptr);
}

void chx01_freqsweep_store_scale_factor(ch_dev_t *dev_ptr) {
	uint8_t err;
	uint8_t tof_sf_reg;
	uint16_t scale_factor;

	tof_sf_reg = CHX01_FREQSWEEP_REG_TOF_SF;

	err = chdrv_read_word(dev_ptr, tof_sf_reg, &scale_factor);
	if (!err) {
		dev_ptr->tof_scale_factor = scale_factor;
	} else {
		dev_ptr->tof_scale_factor = 0;
	}
}

int chx01_freqsweep_set_pulse_width(ch_dev_t *dev_ptr, uint8_t pulse_width) {
	return chdrv_write_byte(dev_ptr, CHX01_FREQSWEEP_REG_PULSE_WIDTH, pulse_width);
}

int chx01_freqsweep_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length) {
	return chdrv_write_byte(dev_ptr, CHX01_FREQSWEEP_REG_TXLENGTH, tx_length);
}

int chx01_freqsweep_set_dco_divider(ch_dev_t *dev_ptr, uint8_t dco_divider) {
	return chdrv_write_byte(dev_ptr, CHX01_FREQSWEEP_REG_DCODIVIDER, dco_divider);
}

int chx01_freqsweep_set_dco_start(ch_dev_t *dev_ptr, uint16_t dco_start) {
	return chdrv_write_word(dev_ptr, CHX01_FREQSWEEP_REG_DCOSTART, dco_start);
}

int chx01_freqsweep_set_dco_stop(ch_dev_t *dev_ptr, uint16_t dco_stop) {
	return chdrv_write_word(dev_ptr, CHX01_FREQSWEEP_REG_DCOSTOP, dco_stop);
}

uint32_t chx01_freqsweep_get_op_freq(ch_dev_t *dev_ptr) {
	uint32_t num, den, opfreq;
	uint16_t raw_freq;  // aka scale factor
	uint32_t freq_counter_cycles = dev_ptr->freqCounterCycles;

	chdrv_read_word(dev_ptr, CHX01_FREQSWEEP_REG_TOF_SF, &raw_freq);
	num    = (uint32_t)(((dev_ptr->rtc_cal_result) * 1000U) / (16U * freq_counter_cycles)) * (uint32_t)(raw_freq);
	den    = dev_ptr->group->rtc_cal_pulse_ms;
	opfreq = (num / den);
	return opfreq;
}

uint16_t chx01_freqsweep_get_dco_code(ch_dev_t *dev_ptr) {
	uint16_t dcoCode = 0;
	if (dev_ptr->sensor_connected) {
		chdrv_read_word(dev_ptr, CHX01_FREQSWEEP_REG_DCOCODE, &dcoCode);
	}
	return dcoCode;
}
