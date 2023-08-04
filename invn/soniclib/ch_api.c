/*! \file ch_api.c
 \brief Chirp SonicLib public API functions for using the Chirp ultrasonic sensor.

 The user should not need to edit this file. This file relies on hardware interface
 functions declared in ch_bsp.h and supplied in the board support package (BSP) for
 the specific hardware platform being used.
 */

/*
 Copyright 2016-2023, InvenSense, Inc.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED.

 */
#include <stdbool.h>

#include <invn/soniclib/soniclib.h>
#include <invn/soniclib/details/ch_common.h>
#include <invn/soniclib/details/ch_driver.h>
#include <invn/soniclib/chirp_bsp.h>
#include <invn/soniclib/details/ch_math_utils.h>

/*!
 * \brief Initialize a Chirp ultrasonic sensor descriptor structure
 *
 * \param dev_ptr 		a pointer to the ch_dev_t config structure for a sensor
 *
 * \return 0 (RET_OK) if successful, non-zero otherwise
 *
 */

uint8_t ch_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t dev_num, ch_fw_init_func_t fw_init_func) {

	uint8_t ret_val = RET_ERR;

	if (fw_init_func != NULL) {
		ret_val = ch_common_init(dev_ptr, grp_ptr, dev_num, fw_init_func);
	}

	if (ret_val == RET_OK) {
		/* Set mode to idle */
		dev_ptr->mode = CH_MODE_IDLE;

#ifdef INCLUDE_SHASTA_SUPPORT
		/* Initialize sensor measurement queue */
		ret_val |= ch_common_meas_init_queue(dev_ptr);
#endif
	}

	return ret_val;
}

uint8_t ch_get_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr) {
	uint8_t ret_val = 0;

	config_ptr->mode            = dev_ptr->mode;
	config_ptr->max_range       = dev_ptr->max_range;
	config_ptr->static_range    = dev_ptr->static_range;
	config_ptr->sample_interval = dev_ptr->freerun_intvl_us / 1000;
	config_ptr->thresh_ptr      = NULL;  // thresholds not returned here - use ch_get_thresholds()

	return ret_val;
}

uint8_t ch_set_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr) {
	uint8_t ret_val = 0;

	ret_val = ch_set_max_range(dev_ptr, config_ptr->max_range);  // set max range

	if (!ret_val) {

		if (dev_ptr->api_funcs.set_static_range != NULL) {                     // if STR supported
			ret_val = ch_set_static_range(dev_ptr, config_ptr->static_range);  // set static target rejection range

			if (!ret_val) {
				dev_ptr->static_range = config_ptr->static_range;
			}
		}
	}

	if (!ret_val) {
		if (config_ptr->sample_interval != 0) {
			ret_val = ch_set_freerun_interval(dev_ptr,
			                                  config_ptr->sample_interval);  // set meas interval (free-run mode only)
		}
	}

	if (!ret_val) {
		dev_ptr->freerun_intvl_us = config_ptr->sample_interval * 1000;    // store  interval in usec
		if (dev_ptr->api_funcs.set_thresholds != NULL) {                   // if multi-thresholds supported
			ret_val = ch_set_thresholds(dev_ptr, config_ptr->thresh_ptr);  // set multiple thresholds
		}
	}

	if (!ret_val) {
		if (dev_ptr->api_funcs.set_target_interrupt != NULL) {  // if target interrupt mode supported
			ret_val = ch_set_target_interrupt(dev_ptr,
			                                  config_ptr->tgt_int_filter);  // enable/disable target detect interrupt
		}
	}

	if (!ret_val) {
		if (dev_ptr->api_funcs.set_time_plan != NULL) {                  // if SonicSync time plans supported
			ret_val = ch_set_time_plan(dev_ptr, config_ptr->time_plan);  // set time plan (sonicsync only)
		}
	}

	if (!ret_val) {
		ret_val = ch_set_mode(dev_ptr, config_ptr->mode);  // set operating mode last
	}

	if (!ret_val) {
		dev_ptr->mode = config_ptr->mode;
	}

	return ret_val;
}

uint8_t ch_group_start(ch_group_t *grp_ptr) {

	return ch_common_group_start(grp_ptr);
}

uint8_t ch_restart(ch_dev_t *dev_ptr) {
	uint8_t ret_val = RET_ERR;
	;

#ifdef INCLUDE_SHASTA_SUPPORT
	ret_val = chdrv_restart(dev_ptr);  // restart & program sensor

	if (!ret_val) {
		ret_val  = ch_meas_write_config(dev_ptr);                       // write meas config to sensor
		ret_val |= chdrv_event_trigger(dev_ptr, EVENT_CONFIG_TRIGGER);  // apply trigger pin selection
	}
#else
	(void)dev_ptr;  // not supported for CH101/CH201, return error
#endif
	return ret_val;
}

uint8_t ch_group_restart(ch_group_t *grp_ptr) {
	uint8_t ret_val = RET_OK;

	for (int dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = grp_ptr->device[dev_num];

		if (dev_ptr->sensor_connected) {
			ret_val |= ch_restart(dev_ptr);
		}
	}
	return ret_val;
}

void ch_trigger(ch_dev_t *dev_ptr) {

	if (dev_ptr->trig_type == CH_TRIGGER_TYPE_HW) {  // hardware trigger
		chdrv_hw_trigger(dev_ptr);
	} else {  // software trigger
		ch_trigger_soft(dev_ptr);
	}
}

void ch_trigger_soft(ch_dev_t *dev_ptr) {
	ch_trigger_soft_func_t func_ptr = dev_ptr->api_funcs.trigger_soft;

	if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
		chdrv_event_trigger(dev_ptr, EVENT_SW_TRIG);
	} else if (func_ptr != NULL) {  // CH101 and CH201 support depends on f/w type
		(*func_ptr)(dev_ptr);
	}
}

void ch_set_trigger_type(ch_dev_t *dev_ptr, ch_trigger_type_t trig_type) {

	if ((dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) || (dev_ptr->api_funcs.trigger_soft != NULL)) {
		dev_ptr->trig_type = trig_type;
	}
}

ch_trigger_type_t ch_get_trigger_type(ch_dev_t *dev_ptr) {
	return dev_ptr->trig_type;
}

void ch_group_trigger(ch_group_t *grp_ptr) {
	chdrv_group_hw_trigger(grp_ptr);  // group trigger is h/w only for now
}

void ch_reset(ch_dev_t *dev_ptr, ch_reset_t reset_type) {

	if (reset_type == CH_RESET_HARD) {
		chdrv_group_hard_reset(dev_ptr->group);  // TODO need single device hard reset
	} else {
		chdrv_soft_reset(dev_ptr);
	}
}

void ch_group_reset(ch_group_t *grp_ptr, ch_reset_t reset_type) {
	if (reset_type == CH_RESET_HARD) {
		chdrv_group_hard_reset(grp_ptr);
	} else {
		chdrv_group_soft_reset(grp_ptr);
	}
}

uint8_t ch_sensor_is_connected(ch_dev_t *dev_ptr) {

	return dev_ptr->sensor_connected;
}

uint16_t ch_get_part_number(ch_dev_t *dev_ptr) {

	return dev_ptr->part_number;
}

uint8_t ch_get_dev_num(ch_dev_t *dev_ptr) {

	return dev_ptr->io_index;
}

ch_dev_t *ch_get_dev_ptr(ch_group_t *grp_ptr, uint8_t dev_num) {

	return grp_ptr->device[dev_num];
}

uint8_t ch_get_i2c_address(ch_dev_t *dev_ptr) {
	uint8_t i2c_address = 0;
#ifdef INCLUDE_WHITNEY_SUPPORT
	i2c_address = dev_ptr->i2c_address;
#else
	(void)dev_ptr;
#endif  // INCLUDE_WHITNEY_SUPPORT
	return i2c_address;
}

uint8_t ch_get_bus(ch_dev_t *dev_ptr) {

	return dev_ptr->bus_index;
}

uint8_t ch_get_num_ports(ch_group_t *grp_ptr) {

	return grp_ptr->num_ports;
}

void ch_get_version(ch_version_t *version_ptr) {
	if (version_ptr != NULL) {
		version_ptr->major = SONICLIB_VER_MAJOR;
		version_ptr->minor = SONICLIB_VER_MINOR;
		version_ptr->rev   = SONICLIB_VER_REV;
	}
}

const char *ch_get_fw_version_string(ch_dev_t *dev_ptr) {

	return dev_ptr->fw_version_string;
}

uint8_t ch_log_init(ch_group_t *grp_ptr, ch_log_fmt_t format, ch_log_cfg_t *config_ptr) {
	static uint8_t log_id = 0;
	uint16_t num_samples_no_decim;
	uint8_t start_sample_no_decim;

	if (format == CH_LOG_FMT_REDSWALLOW) {
		printf("# TDK InvenSense Embedded Redswallow Log\r\n");
		printf("# sample rate: %0.2f S/s\r\n", (float)1000.0f / config_ptr->interval_ms);
		printf("# Decimation factor: %u\r\n", config_ptr->decimation_factor);
		printf("# Content: %s\r\n", config_ptr->output_type == CH_OUTPUT_IQ ? "iq" : "amp");

		for (uint8_t dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
			ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

			if (ch_sensor_is_connected(dev_ptr)) {
				if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
					num_samples_no_decim  = dev_ptr->num_rx_samples;
					start_sample_no_decim = config_ptr->start_sample;
				} else {
					/* on shasta samples returned are already decimated */
					num_samples_no_decim  = dev_ptr->num_rx_samples * config_ptr->decimation_factor;
					start_sample_no_decim = config_ptr->start_sample * config_ptr->decimation_factor;
				}
				printf("# Sensors ID: %u\r\n", dev_num);
				printf("# Sensors FOP: %lu Hz\r\n", dev_ptr->op_frequency);
				printf("# Sensors NB Samples: %u\r\n", num_samples_no_decim);
				printf("# Sensors NB First samples skipped: %u\r\n", start_sample_no_decim);
				printf("# header, log ID, time [s], tx_id, rx_id, range [cm], intensity [a.u.], target_detected, "
				       "insertionAnnotation");

				for (int count  = start_sample_no_decim; count < num_samples_no_decim;
				     count     += config_ptr->decimation_factor) {
					float sample_cm = (float)dev_ptr->max_range * (count + 1) / (num_samples_no_decim * 10.0);
					printf(", idata_%0.1f", sample_cm);
				}
				for (int count  = start_sample_no_decim; count < num_samples_no_decim;
				     count     += config_ptr->decimation_factor) {
					float sample_cm = (float)dev_ptr->max_range * (count + 1) / (num_samples_no_decim * 10.0);
					printf(", qdata_%0.1f", sample_cm);
				}
				printf("\r\n");
			}
		}
	}
	return ++log_id;
}

void ch_log_append(uint8_t log_id, ch_log_fmt_t format, uint64_t timestamp, ch_log_data_t *log_data_ptr) {
	if (format == CH_LOG_FMT_REDSWALLOW) {
		int16_t q_data[CH201_MAX_NUM_SAMPLES];

		printf("chlog, %d, %0.6f, %u, %u, %0.1f, %u, %u, %u", log_id, (float)timestamp / 1000000.0f,
		       log_data_ptr->tx_sensor_id, log_data_ptr->rx_sensor_id, (float)log_data_ptr->range / (32.0f * 10.0f),
		       log_data_ptr->amplitude, log_data_ptr->range != CH_NO_TARGET, log_data_ptr->annotation);

		/* Print first all I data then all Q data
		 * If printing magnitude, print mag data in place of I data and 0 in place of Q data
		 * so sqrt(I^2+Q^2) is always possible whatever the input format used */
		for (int count = 0; count < log_data_ptr->num_samples; count++) {
			if (log_data_ptr->output_type == CH_OUTPUT_IQ) {
				/* print I data now */
				printf(", %d", log_data_ptr->raw_data.iq_sample_ptr->i);
				/* save Q data to be print after */
				q_data[count] = log_data_ptr->raw_data.iq_sample_ptr->q;
				log_data_ptr->raw_data.iq_sample_ptr++;
			} else {
				printf(", %u", *(log_data_ptr->raw_data.mag_data_ptr));
				q_data[count] = 0;
				log_data_ptr->raw_data.mag_data_ptr++;
			}
		}
		/* Q data */
		for (int count = 0; count < log_data_ptr->num_samples; count++) {
			printf(", %d", q_data[count]);
		}
		printf("\r\n");
	}
}

ch_mode_t ch_get_mode(ch_dev_t *dev_ptr) {

	return dev_ptr->mode;
}

uint8_t ch_set_mode(ch_dev_t *dev_ptr, ch_mode_t mode) {
	int ret_val                 = RET_ERR;
	ch_set_mode_func_t func_ptr = dev_ptr->api_funcs.set_mode;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, mode);
	}

	if (ret_val == 0) {
		dev_ptr->mode = mode;
	}

	return ret_val;
}

uint16_t ch_get_sample_interval(ch_dev_t *dev_ptr) {
	/* Deprecated - use new equivalent function */
	return ch_get_freerun_interval(dev_ptr);
}

uint16_t ch_get_freerun_interval(ch_dev_t *dev_ptr) {
	uint16_t interval_ms = 0;

	if (dev_ptr->mode == CH_MODE_FREERUN) {
		interval_ms = (dev_ptr->freerun_intvl_us / 1000);
	}
	return interval_ms;
}

uint32_t ch_get_freerun_interval_us(ch_dev_t *dev_ptr) {
	uint32_t interval_us = 0;

	if (dev_ptr->mode == CH_MODE_FREERUN) {
		interval_us = dev_ptr->freerun_intvl_us;
	}
	return interval_us;
}

uint32_t ch_get_freerun_interval_ticks(ch_dev_t *dev_ptr) {
	uint32_t rtc_ticks = 0;

	if (dev_ptr->mode == CH_MODE_FREERUN) {
		rtc_ticks = ch_usec_to_ticks(dev_ptr, dev_ptr->freerun_intvl_us);
	}
	return rtc_ticks;
}

uint8_t ch_set_freerun_interval(ch_dev_t *dev_ptr, uint16_t sense_interval_ms) {

	return ch_common_set_freerun_interval(dev_ptr, sense_interval_ms);
}

uint8_t ch_set_freerun_interval_us(ch_dev_t *dev_ptr, uint32_t sense_interval_us) {

	return ch_common_set_freerun_interval_us(dev_ptr, sense_interval_us);
}

uint8_t ch_set_freerun_interval_ticks(ch_dev_t *dev_ptr, uint32_t sense_interval_ticks) {

	return ch_common_set_freerun_interval_ticks(dev_ptr, sense_interval_ticks);
}

uint8_t ch_set_sample_interval(ch_dev_t *dev_ptr, uint16_t sample_interval) {
	/* Deprecated - use new equivalent routine */
	return ch_set_freerun_interval(dev_ptr, sample_interval);
}

uint8_t ch_freerun_time_hop_enable(ch_dev_t *dev_ptr) {
	return ch_common_set_freerun_time_hop(dev_ptr, true);
}

uint8_t ch_freerun_time_hop_disable(ch_dev_t *dev_ptr) {
	return ch_common_set_freerun_time_hop(dev_ptr, false);
}

uint16_t ch_get_num_samples(ch_dev_t *dev_ptr) {

	return ch_common_get_num_samples(dev_ptr);
}

uint8_t ch_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t ret_val                    = RET_ERR;
	ch_set_num_samples_func_t func_ptr = dev_ptr->api_funcs.set_num_samples;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	if (!ret_val) {
		dev_ptr->max_range = ch_samples_to_mm(dev_ptr, num_samples);  // store corresponding range in mm
	}
	return ret_val;
}

uint16_t ch_get_max_range(ch_dev_t *dev_ptr) {

	return dev_ptr->max_range;
}

uint8_t ch_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range) {
	uint8_t ret_val                  = RET_ERR;
	ch_set_max_range_func_t func_ptr = dev_ptr->api_funcs.set_max_range;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, max_range);
	}

	return ret_val;
}

uint16_t ch_get_max_samples(ch_dev_t *dev_ptr) {

	return dev_ptr->max_samples;
}

uint8_t ch_get_sample_window(ch_dev_t *dev_ptr, uint16_t *start_sample_ptr, uint16_t *num_samples_ptr) {
	uint8_t ret_val = RET_ERR;

	if ((start_sample_ptr != NULL) && (num_samples_ptr != NULL)) {

		*start_sample_ptr = dev_ptr->win_start_sample;
		*num_samples_ptr  = dev_ptr->num_win_samples;
		ret_val           = RET_OK;
	}
	return ret_val;
}

uint8_t ch_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t num_samples) {
	uint8_t ret_val                      = RET_ERR;
	ch_set_sample_window_func_t func_ptr = dev_ptr->api_funcs.set_sample_window;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, start_sample, num_samples);
	}

	return ret_val;
}

uint16_t ch_get_static_range(ch_dev_t *dev_ptr) {

	return dev_ptr->static_range;
}

uint8_t ch_set_static_range(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t ret_val                     = RET_OK;
	ch_set_static_range_func_t func_ptr = dev_ptr->api_funcs.set_static_range;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	return ret_val;
}

uint32_t ch_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint32_t range               = 0;
	ch_get_range_func_t func_ptr = dev_ptr->api_funcs.get_range;

	if (func_ptr != NULL) {
		range = (*func_ptr)(dev_ptr, range_type);
	}

	return range;
}

uint32_t ch_get_tof_us(ch_dev_t *dev_ptr) {
	uint32_t tof_us               = 0;
	ch_get_tof_us_func_t func_ptr = dev_ptr->api_funcs.get_tof_us;

	if (func_ptr != NULL) {
		tof_us = (*func_ptr)(dev_ptr);
	}

	return tof_us;
}

uint16_t ch_get_amplitude(ch_dev_t *dev_ptr) {
	int amplitude                    = 0;
	ch_get_amplitude_func_t func_ptr = dev_ptr->api_funcs.get_amplitude;

	if (func_ptr != NULL) {
		amplitude = (*func_ptr)(dev_ptr);
	}

	return amplitude;
}

uint16_t ch_get_amplitude_avg(ch_dev_t *dev_ptr) {
	uint16_t amplitude_avg               = 0;
	ch_get_amplitude_avg_func_t func_ptr = dev_ptr->api_funcs.get_amplitude_avg;

	if (func_ptr != NULL) {
		amplitude_avg = (*func_ptr)(dev_ptr);
	}

	return amplitude_avg;
}

uint8_t ch_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *amp_buf_ptr, uint16_t start_sample, uint16_t num_samples,
                              ch_io_mode_t mode) {
	uint16_t error                        = RET_ERR;
	ch_get_amplitude_data_func_t func_ptr = dev_ptr->api_funcs.get_amplitude_data;

	if (func_ptr != NULL) {
		error = (*func_ptr)(dev_ptr, amp_buf_ptr, start_sample, num_samples, mode);
	}

	return error;
}

uint8_t ch_get_amp_thresh_data(ch_dev_t *dev_ptr, ch_amp_thresh_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
                               ch_io_mode_t mode) {

	return ch_common_get_amp_thresh_data(dev_ptr, buf_ptr, start_sample, num_samples, mode);
}

uint16_t ch_get_bandwidth(ch_dev_t *dev_ptr) {

	return dev_ptr->bandwidth;
}

uint8_t ch_set_frequency(ch_dev_t *dev_ptr, uint32_t request_op_freq_hz) {
	int ret_val                      = RET_ERR;
	ch_set_frequency_func_t func_ptr = dev_ptr->api_funcs.set_frequency;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, request_op_freq_hz);
#ifdef INCLUDE_SHASTA_SUPPORT
	} else {
		if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
			ret_val = ch_common_set_frequency(dev_ptr, request_op_freq_hz);
		}
#endif
	}

	return ret_val;
}

uint32_t ch_get_frequency(ch_dev_t *dev_ptr) {

	return dev_ptr->op_frequency;
}

uint8_t ch_group_set_frequency(ch_group_t *grp_ptr, uint32_t request_op_freq_hz) {

	return ch_common_group_set_frequency(grp_ptr, request_op_freq_hz);
}

uint32_t ch_group_get_frequency(ch_group_t *grp_ptr) {

	return grp_ptr->op_frequency;
}

uint16_t ch_get_rtc_cal_pulselength(ch_dev_t *dev_ptr) {

	return dev_ptr->group->rtc_cal_pulse_ms;
}

uint16_t ch_get_rtc_cal_result(ch_dev_t *dev_ptr) {

	return dev_ptr->rtc_cal_result;
}

uint16_t ch_get_scale_factor(ch_dev_t *dev_ptr) {

	return dev_ptr->amp_scale_factor;
}

uint8_t ch_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
                       ch_io_mode_t mode) {
	int ret_val                    = RET_ERR;
	ch_get_iq_data_func_t func_ptr = dev_ptr->api_funcs.get_iq_data;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, buf_ptr, start_sample, num_samples, mode);
	}

	return ret_val;
}

uint16_t ch_samples_to_mm(ch_dev_t *dev_ptr, uint16_t num_samples) {
	int num_mm                       = 0;
	ch_samples_to_mm_func_t func_ptr = dev_ptr->api_funcs.samples_to_mm;

	if (func_ptr != NULL) {
		num_mm = (*func_ptr)(dev_ptr, num_samples);
	}

	return num_mm;
}

uint16_t ch_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm) {
	int num_samples                  = 0;
	ch_mm_to_samples_func_t func_ptr = dev_ptr->api_funcs.mm_to_samples;

	if (func_ptr != NULL) {
		num_samples = (*func_ptr)(dev_ptr, num_mm);
	}

	return num_samples;
}

uint8_t ch_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude) {
	int ret_val                      = RET_ERR;
	ch_set_threshold_func_t func_ptr = dev_ptr->api_funcs.set_threshold;

	if ((func_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, threshold_index, amplitude);
	}

	return ret_val;
}

uint16_t ch_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index) {
	uint16_t amplitude               = 0;
	ch_get_threshold_func_t func_ptr = dev_ptr->api_funcs.get_threshold;

	if ((func_ptr != NULL)) {
		amplitude = (*func_ptr)(dev_ptr, threshold_index);
	}

	return amplitude;
}

uint16_t ch_iq_to_amplitude(ch_iq_sample_t *iq_sample) {
	uint32_t amplitude;
	uint32_t i_sq = ((int32_t)iq_sample->i * (int32_t)iq_sample->i);
	uint32_t q_sq = ((int32_t)iq_sample->q * (int32_t)iq_sample->q);

	amplitude = sqrt_int32(i_sq + q_sq);

	return (uint16_t)amplitude;
}

uint8_t ch_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr) {
	int ret_val = RET_ERR;

#ifdef INCLUDE_ALGO_RANGEFINDER
	ch_set_thresholds_func_t func_ptr = dev_ptr->api_funcs.set_thresholds;

	if ((func_ptr != NULL) && (thresh_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, thresh_ptr);
	}
#else
	(void)dev_ptr;
	(void)thresh_ptr;
#endif

	return ret_val;
}

uint8_t ch_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr) {
	int ret_val = RET_ERR;

#ifdef INCLUDE_ALGO_RANGEFINDER
	ch_get_thresholds_func_t func_ptr = dev_ptr->api_funcs.get_thresholds;

	if ((func_ptr != NULL) && (thresh_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, thresh_ptr);
	}
#else
	(void)dev_ptr;
	(void)thresh_ptr;
#endif

	return ret_val;
}

uint8_t ch_get_num_thresholds(ch_dev_t *dev_ptr) {
	uint8_t num_thresholds = 0;

#ifdef INCLUDE_ALGO_RANGEFINDER
	num_thresholds = dev_ptr->max_num_thresholds;
#else
	(void)dev_ptr;
#endif

	return num_thresholds;
}

uint8_t ch_set_time_plan(ch_dev_t *dev_ptr, ch_time_plan_t time_plan) {
	uint8_t ret_val                  = RET_ERR;
	ch_set_time_plan_func_t func_ptr = dev_ptr->api_funcs.set_time_plan;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, time_plan);
	}

	return ret_val;
}

ch_time_plan_t ch_get_time_plan(ch_dev_t *dev_ptr) {
	ch_time_plan_t time_plan         = CH_TIME_PLAN_NONE;
	ch_get_time_plan_func_t func_ptr = dev_ptr->api_funcs.get_time_plan;

	if (func_ptr != NULL) {
		time_plan = (*func_ptr)(dev_ptr);
	}

	return time_plan;
}

/*!
 * \brief Start a non-blocking sensor readout
 *
 * \param grp_ptr 		pointer to the ch_group_t descriptor structure for a group of sensors
 *
 * This function starts a non-blocking I/O operation on the specified group of sensors.
 */
uint8_t ch_io_start_nb(ch_group_t *grp_ptr) {
	uint8_t ret_val = 1;

	// only start I/O if there is a callback function
	if (grp_ptr->io_complete_callback != NULL) {
		chdrv_group_start_nb(grp_ptr);
		ret_val = 0;
	}

	return ret_val;
}

/*!
 * \brief Notify SonicLib that a sensor interrupt occurred.
 *
 * This function is used from the board support package (BSP) handler routine
 * for the sensor interrupt to notify SonicLib that the interrupt has
 * occurred.  The SonicLib driver layer will handle further processing of
 * the interrupt, including a call to the application-supplied callback
 * function.
 */
void ch_interrupt(ch_group_t *grp_ptr, uint8_t dev_num) {

	chdrv_int_callback(grp_ptr, dev_num);
}

/*!
 * \brief Set callback function for Chirp sensor I/O interrupt
 *
 * \note
 */
void ch_io_int_callback_set(ch_group_t *grp_ptr, ch_io_int_callback_t callback_func_ptr) {

	grp_ptr->io_int_callback = callback_func_ptr;
}

/*!
 * \brief Set callback function for Chirp sensor I/O operation complete
 *
 * \note
 */
void ch_io_complete_callback_set(ch_group_t *grp_ptr, ch_io_complete_callback_t callback_func_ptr) {

	grp_ptr->io_complete_callback = callback_func_ptr;
}

/*!
 * \brief Continue a non-blocking readout
 *
 * \param grp_ptr 			pointer to the ch_group_t config structure for a group of sensors
 * \param i2c_bus_index		index value identifying I2C/SPI bus within group
 *
 * Call this function once from your I2C interrupt handler each time it completes an I/O operation.
 * It will call the function previously specified during \a ch_io_complete_callback_set() when all group
 * transactions are complete.
 */
void ch_io_notify(ch_group_t *grp_ptr, uint8_t i2c_bus_index) {

	chdrv_group_irq_handler(grp_ptr, i2c_bus_index);
}

uint8_t ch_set_target_interrupt(ch_dev_t *dev_ptr, ch_tgt_int_filter_t tgt_int_filter) {
	int ret_val                             = RET_ERR;
	ch_set_target_interrupt_func_t func_ptr = dev_ptr->api_funcs.set_target_interrupt;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, tgt_int_filter);
	}

	return ret_val;
}

ch_tgt_int_filter_t ch_get_target_interrupt(ch_dev_t *dev_ptr) {
	ch_tgt_int_filter_t tgt_int_mode        = CH_TGT_INT_FILTER_OFF;
	ch_get_target_interrupt_func_t func_ptr = dev_ptr->api_funcs.get_target_interrupt;

	if (func_ptr != NULL) {
		tgt_int_mode = (*func_ptr)(dev_ptr);
	}

	return tgt_int_mode;
}

uint8_t ch_set_target_int_counter(ch_dev_t *dev_ptr, uint8_t meas_hist, uint8_t thresh_count, uint8_t reset) {
	uint8_t ret_val                           = RET_ERR;
	ch_set_target_int_counter_func_t func_ptr = dev_ptr->api_funcs.set_target_int_counter;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, meas_hist, thresh_count, reset);
	}

	return ret_val;
}

uint8_t ch_get_target_int_counter(ch_dev_t *dev_ptr, uint8_t *meas_hist_ptr, uint8_t *thresh_count_ptr,
                                  uint8_t *reset_ptr) {
	uint8_t ret_val                           = RET_ERR;
	ch_get_target_int_counter_func_t func_ptr = dev_ptr->api_funcs.get_target_int_counter;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, meas_hist_ptr, thresh_count_ptr, reset_ptr);
	}

	return ret_val;
}

#ifdef INCLUDE_SHASTA_SUPPORT
uint8_t ch_set_interrupt_mode(ch_dev_t *dev_ptr, ch_interrupt_mode_t mode) {

	return ch_common_set_interrupt_mode(dev_ptr, mode);
}

ch_interrupt_mode_t ch_get_interrupt_mode(ch_dev_t *dev_ptr) {

	return ch_common_get_interrupt_mode(dev_ptr);
}
#endif

uint8_t ch_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff) {
	int ret_val                         = RET_ERR;
	ch_set_static_coeff_func_t func_ptr = dev_ptr->api_funcs.set_static_coeff;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, static_coeff);
	}

	return ret_val;
}

uint8_t ch_get_static_coeff(ch_dev_t *dev_ptr) {
	uint8_t statc_coeff                 = 0;
	ch_get_static_coeff_func_t func_ptr = dev_ptr->api_funcs.get_static_coeff;

	if (func_ptr != NULL) {
		statc_coeff = (*func_ptr)(dev_ptr);
	}

	return statc_coeff;
}

uint8_t ch_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples) {
	int ret_val                       = RET_ERR;
	ch_set_rx_holdoff_func_t func_ptr = dev_ptr->api_funcs.set_rx_holdoff;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	return ret_val;
}

uint16_t ch_get_rx_holdoff(ch_dev_t *dev_ptr) {
	uint16_t num_samples              = 0;
	ch_get_rx_holdoff_func_t func_ptr = dev_ptr->api_funcs.get_rx_holdoff;

	if (func_ptr != NULL) {
		num_samples = (*func_ptr)(dev_ptr);
	}

	return num_samples;
}

uint8_t ch_set_rx_low_gain(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t ret_val = RET_ERR;

	ch_set_rx_low_gain_func_t func_ptr = dev_ptr->api_funcs.set_rx_low_gain;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	return ret_val;
}

uint16_t ch_get_rx_low_gain(ch_dev_t *dev_ptr) {
	uint16_t num_samples = 0;

	ch_get_rx_low_gain_func_t func_ptr = dev_ptr->api_funcs.get_rx_low_gain;

	if (func_ptr != NULL) {
		num_samples = (*func_ptr)(dev_ptr);
	}

	return num_samples;
}

uint8_t ch_set_tx_length(ch_dev_t *dev_ptr, uint16_t tx_length) {
	int ret_val                      = RET_ERR;
	ch_set_tx_length_func_t func_ptr = dev_ptr->api_funcs.set_tx_length;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, tx_length);
	}

	return ret_val;
}

uint16_t ch_get_tx_length(ch_dev_t *dev_ptr) {
	uint16_t tx_length               = 0;
	ch_get_tx_length_func_t func_ptr = dev_ptr->api_funcs.get_tx_length;

	if (func_ptr != NULL) {
		tx_length = (*func_ptr)(dev_ptr);
	}

	return tx_length;
}

uint8_t ch_get_rx_pulse_length(ch_dev_t *dev_ptr) {
	uint8_t rx_pulse_length                = 0;
	ch_get_rx_pulse_length_func_t func_ptr = dev_ptr->api_funcs.get_rx_pulse_length;

	if (func_ptr != NULL) {
		rx_pulse_length = (*func_ptr)(dev_ptr);
	}

	return rx_pulse_length;
}

void ch_set_rx_pretrigger(ch_group_t *grp_ptr, uint8_t enable) {

	if (enable) {
		chdrv_pretrigger_delay_set(grp_ptr, CHDRV_PRETRIGGER_DELAY_US);
	} else {
		chdrv_pretrigger_delay_set(grp_ptr, 0);
	}
}

uint8_t ch_get_rx_pretrigger(ch_group_t *grp_ptr) {
	uint8_t enabled = (grp_ptr->pretrig_delay_us != 0);

	return enabled;
}

uint8_t ch_check_program(ch_dev_t *dev_ptr) {

	return ch_common_check_program(dev_ptr);
}

uint8_t ch_set_modulated_tx_data(ch_dev_t *dev_ptr, uint8_t tx_data) {
	int ret_val                              = RET_ERR;
	ch_set_modulated_tx_data_func_t func_ptr = dev_ptr->api_funcs.set_modulated_tx_data;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, tx_data);
	}

	return ret_val;
}

uint8_t ch_get_demodulated_rx_data(ch_dev_t *dev_ptr, uint8_t rx_pulse_length, uint8_t *data_ptr) {
	int ret_val                                = RET_ERR;
	ch_get_demodulated_rx_data_func_t func_ptr = dev_ptr->api_funcs.get_demodulated_rx_data;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, rx_pulse_length, data_ptr);
	}

	return ret_val;
}

uint8_t ch_set_cal_result(ch_dev_t *dev_ptr, ch_cal_result_t *cal_ptr) {
	int ret_val                       = RET_ERR;
	ch_set_cal_result_func_t func_ptr = dev_ptr->api_funcs.set_cal_result;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, cal_ptr);
	}

	return ret_val;
}

uint8_t ch_get_cal_result(ch_dev_t *dev_ptr, ch_cal_result_t *cal_ptr) {
	int ret_val                       = RET_ERR;
	ch_get_cal_result_func_t func_ptr = dev_ptr->api_funcs.get_cal_result;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, cal_ptr);
	}

	return ret_val;
}

uint8_t ch_set_data_output(ch_dev_t *dev_ptr, ch_output_t *output_ptr) {
	int ret_val                        = RET_ERR;
	ch_set_data_output_func_t func_ptr = dev_ptr->api_funcs.set_data_output;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, output_ptr);
	}

	return ret_val;
}

#ifdef INCLUDE_SHASTA_SUPPORT
uint8_t ch_meas_init_queue(ch_dev_t *dev_ptr) {

	return ch_common_meas_init_queue(dev_ptr);
}

uint8_t ch_meas_reset(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_reset(dev_ptr, meas_num);
}

uint8_t ch_meas_init(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_config_t *meas_config_ptr,
                     ch_thresholds_t *thresh_ptr) {

	return ch_common_meas_init(dev_ptr, meas_num, meas_config_ptr, thresh_ptr);
}
#endif

ch_meas_status_t ch_meas_get_status(ch_dev_t *dev_ptr, uint8_t meas_num) {
	ch_meas_status_t status            = CH_MEAS_STATUS_UNKNOWN;
	ch_meas_get_status_func_t func_ptr = dev_ptr->api_funcs.meas_get_status;

	if (func_ptr != NULL) {
		status = (*func_ptr)(dev_ptr, meas_num);
	}
	return status;
}

uint8_t ch_set_data_ready_delay(ch_dev_t *dev_ptr, uint8_t delay_ms) {
	int ret_val                             = RET_ERR;
	ch_set_data_ready_delay_func_t func_ptr = dev_ptr->api_funcs.set_data_ready_delay;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, delay_ms);
	}

	return ret_val;
}

uint8_t ch_get_data_ready_delay(ch_dev_t *dev_ptr) {
	uint8_t delay_ms                        = 0;
	ch_get_data_ready_delay_func_t func_ptr = dev_ptr->api_funcs.get_data_ready_delay;

	if (func_ptr != NULL) {
		delay_ms = (*func_ptr)(dev_ptr);
	}

	return delay_ms;
}

#ifdef INCLUDE_SHASTA_SUPPORT

const char *ch_get_sensor_id(ch_dev_t *dev_ptr) {

	return &(dev_ptr->id_string[0]);
}

uint8_t ch_get_mfg_info(ch_dev_t *dev_ptr, ch_mfg_info_t *info_ptr) {

	return ch_common_get_mfg_info(dev_ptr, info_ptr);
}

uint8_t ch_meas_import(ch_dev_t *dev_ptr, measurement_queue_t *meas_queue_ptr, void *algo_cfg_ptr) {

	return ch_common_meas_import(dev_ptr, meas_queue_ptr, algo_cfg_ptr);
}

uint8_t ch_meas_add_segment(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_segment_t *seg_ptr) {

	return ch_common_meas_add_segment(dev_ptr, meas_num, seg_ptr);
}

uint8_t ch_meas_add_segment_count(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_cycles, uint8_t int_enable) {

	return ch_common_meas_add_segment_count(dev_ptr, meas_num, num_cycles, int_enable);
}

uint8_t ch_meas_add_segment_rx(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t length, uint8_t gain, uint8_t atten,
                               uint8_t int_enable) {

	return ch_common_meas_add_segment_rx(dev_ptr, meas_num, length, gain, atten, int_enable);
}

uint8_t ch_meas_add_segment_tx(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_cycles, uint8_t pulse_width,
                               uint8_t phase, uint8_t int_enable) {

	return ch_common_meas_add_segment_tx(dev_ptr, meas_num, num_cycles, pulse_width, phase, int_enable);
}

void ch_meas_init_segment_count(ch_meas_segment_t *seg_ptr, uint16_t num_cycles, uint8_t int_enable) {

	ch_common_meas_init_segment_count(seg_ptr, num_cycles, int_enable);
}

void ch_meas_init_segment_rx(ch_meas_segment_t *seg_ptr, uint16_t num_samples, ch_odr_t odr, uint8_t gain,
                             uint8_t atten, uint8_t int_enable) {

	ch_common_meas_init_segment_rx(seg_ptr, num_samples, odr, gain, atten, int_enable);
}

void ch_meas_init_segment_tx(ch_meas_segment_t *seg_ptr, uint16_t num_cycles, uint8_t pulse_width, uint8_t phase,
                             uint8_t int_enable) {

	ch_common_meas_init_segment_tx(seg_ptr, num_cycles, pulse_width, phase, int_enable);
}

uint8_t ch_meas_optimize(ch_dev_t *dev_ptr, measurement_queue_t *meas_queue_ptr, void *algo_cfg_ptr) {
	uint8_t ret_val = RET_ERR;

	if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {  // not supported for CH101/CH201
		ret_val = ch_common_meas_optimize(dev_ptr, meas_queue_ptr, algo_cfg_ptr);
	}
	return ret_val;
}

void ch_meas_activate(ch_dev_t *dev_ptr, uint8_t meas_num) {

	ch_common_meas_activate(dev_ptr, meas_num);
}

void ch_meas_standby(ch_dev_t *dev_ptr, uint8_t meas_num) {

	ch_common_meas_standby(dev_ptr, meas_num);
}

uint8_t ch_meas_switch(ch_dev_t *dev_ptr) {

	return ch_common_meas_switch(dev_ptr);
}

uint8_t ch_meas_get_last_num(ch_dev_t *dev_ptr) {

	return ch_common_meas_get_last_num(dev_ptr);
}

uint8_t ch_meas_write_config(ch_dev_t *dev_ptr) {

	return ch_common_meas_write_config(dev_ptr);
}

uint8_t ch_meas_get_queue(ch_dev_t *dev_ptr, measurement_queue_t *meas_queue_ptr) {
	uint8_t ret_val = RET_ERR;

	if (meas_queue_ptr != NULL) {
		/* Copy internal queue definition */
		memcpy((void *)meas_queue_ptr, (void *)&(dev_ptr->meas_queue), sizeof(measurement_queue_t));
		ret_val = RET_OK;
	}
	return ret_val;
}

void ch_meas_get_info(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_info_t *info_ptr) {

	ch_common_meas_get_info(dev_ptr, meas_num, info_ptr);
}

void ch_meas_get_queue_info(ch_dev_t *dev_ptr, ch_meas_queue_info_t *info_ptr) {

	ch_common_meas_get_queue_info(dev_ptr, info_ptr);
}

void ch_meas_get_seg_info(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t seg_num, ch_meas_seg_info_t *info_ptr) {

	ch_common_meas_get_seg_info(dev_ptr, meas_num, seg_num, info_ptr);
}

uint8_t ch_meas_set_interval(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t interval_ms) {

	return ch_common_meas_set_interval(dev_ptr, meas_num, interval_ms);
}

uint8_t ch_meas_set_interval_us(ch_dev_t *dev_ptr, uint8_t meas_num, uint32_t interval_us) {

	return ch_common_meas_set_interval_us(dev_ptr, meas_num, interval_us);
}

uint8_t ch_meas_set_interval_ticks(ch_dev_t *dev_ptr, uint8_t meas_num, uint32_t rtc_ticks) {

	return ch_common_meas_set_interval_ticks(dev_ptr, meas_num, rtc_ticks);
}

uint16_t ch_meas_get_interval(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_get_interval(dev_ptr, meas_num);
}

uint32_t ch_meas_get_interval_us(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_get_interval_us(dev_ptr, meas_num);
}

uint32_t ch_meas_get_interval_ticks(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_get_interval_ticks(dev_ptr, meas_num);
}

uint8_t ch_meas_set_odr(ch_dev_t *dev_ptr, uint8_t meas_num, ch_odr_t odr) {

	return ch_common_meas_set_odr(dev_ptr, meas_num, odr);
}

ch_odr_t ch_meas_get_odr(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_get_odr(dev_ptr, meas_num);
}

uint8_t ch_meas_set_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t num_ranges) {
	uint8_t err = 1;
#ifdef INCLUDE_ALGO_RANGEFINDER
	err = ch_common_meas_set_num_ranges(dev_ptr, meas_num, num_ranges);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)num_ranges;
#endif
	return err;
}

uint8_t ch_meas_get_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint8_t num_ranges = 0;
#ifdef INCLUDE_ALGO_RANGEFINDER
	num_ranges = ch_common_meas_get_num_ranges(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return num_ranges;
}

uint8_t ch_meas_set_num_samples(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {

	return ch_common_meas_set_num_samples(dev_ptr, meas_num, num_samples);
}

uint16_t ch_meas_get_num_samples(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return ch_common_meas_get_num_samples(dev_ptr, meas_num);
}

uint8_t ch_meas_set_max_range(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t max_range_mm) {

	return ch_common_meas_set_max_range(dev_ptr, meas_num, max_range_mm);
}

uint16_t ch_meas_get_max_range(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return dev_ptr->meas_max_range_mm[meas_num];
}

uint16_t ch_meas_mm_to_samples(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_mm) {

	return ch_common_meas_mm_to_samples(dev_ptr, meas_num, num_mm);
}

uint16_t ch_meas_samples_to_mm(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {

	return ch_common_meas_samples_to_mm(dev_ptr, meas_num, num_samples);
}

uint8_t ch_meas_set_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr) {
	uint8_t ret_val = RET_ERR;
#ifdef INCLUDE_ALGO_RANGEFINDER
	ret_val = ch_common_meas_set_thresholds(dev_ptr, meas_num, lib_thresh_buf_ptr);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)lib_thresh_buf_ptr;
#endif
	return ret_val;
}

uint8_t ch_meas_get_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr) {
	uint8_t ret_val = RET_ERR;
#ifdef INCLUDE_ALGO_RANGEFINDER
	ret_val = ch_common_meas_get_thresholds(dev_ptr, meas_num, lib_thresh_buf_ptr);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)lib_thresh_buf_ptr;
#endif
	return ret_val;
}

uint8_t ch_meas_set_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {
	uint8_t ret_val = RET_ERR;
#ifdef INCLUDE_ALGO_RANGEFINDER
	ret_val = ch_common_meas_set_rx_holdoff(dev_ptr, meas_num, num_samples);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)num_samples;
#endif
	return ret_val;
}

uint16_t ch_meas_get_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint16_t num_samples = 0;
#ifdef INCLUDE_ALGO_RANGEFINDER
	num_samples = ch_common_meas_get_rx_holdoff(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return num_samples;
}

uint8_t ch_meas_set_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {
	uint8_t err = 1;
#ifdef INCLUDE_ALGO_RANGEFINDER
	err = ch_common_meas_set_ringdown_cancel(dev_ptr, meas_num, num_samples);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)num_samples;
#endif
	return err;
}

uint16_t ch_meas_get_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint16_t ringdown_cancel = 0;
#ifdef INCLUDE_ALGO_RANGEFINDER
	ringdown_cancel = ch_common_meas_get_ringdown_cancel(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return ringdown_cancel;
}

uint8_t ch_meas_set_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {
	uint8_t err = 1;
#ifdef INCLUDE_ALGO_RANGEFINDER
	err = ch_common_meas_set_static_filter(dev_ptr, meas_num, num_samples);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)num_samples;
#endif
	return err;
}

uint16_t ch_meas_get_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint16_t static_filter = 0;
#ifdef INCLUDE_ALGO_RANGEFINDER
	static_filter = ch_common_meas_get_static_filter(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return static_filter;
}

uint8_t ch_meas_set_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num, ch_output_type_t output_format) {
	uint8_t err = 1;
#ifdef INCLUDE_ALGO_RANGEFINDER
	err = ch_common_meas_set_iq_output(dev_ptr, meas_num, output_format);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)output_format;
#endif
	return err;
}

ch_output_type_t ch_meas_get_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num) {
	ch_output_type_t iq_output = CH_OUTPUT_IQ;  // standard I/Q by default
#ifdef INCLUDE_ALGO_RANGEFINDER
	iq_output = ch_common_meas_get_iq_output(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return iq_output;
}

uint8_t ch_meas_set_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t update_interval) {
	uint8_t err = 1;
#ifdef INCLUDE_ALGO_RANGEFINDER
	err = ch_common_meas_set_filter_update(dev_ptr, meas_num, update_interval);
#else
	(void)dev_ptr;
	(void)meas_num;
	(void)update_interval;
#endif
	return err;
}

uint8_t ch_meas_get_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint8_t filter_update = 0;
#ifdef INCLUDE_ALGO_RANGEFINDER
	filter_update = ch_common_meas_get_filter_update(dev_ptr, meas_num);
#else
	(void)dev_ptr;
	(void)meas_num;
#endif
	return filter_update;
}

uint8_t ch_init_algo(ch_dev_t *dev_ptr) {

	return ch_common_init_algo(dev_ptr);
}

uint8_t ch_get_algo_info(ch_dev_t *dev_ptr, ICU_ALGO_SHASTA_INFO *algo_info_ptr) {

	return ch_common_get_algo_info(dev_ptr, algo_info_ptr);
}

uint8_t ch_get_algo_config(ch_dev_t *dev_ptr, void *algo_cfg_ptr) {

	return ch_common_get_algo_config(dev_ptr, algo_cfg_ptr);
}

uint8_t ch_get_algo_output(ch_dev_t *dev_ptr, void *algo_out_ptr) {

	return ch_common_get_algo_output(dev_ptr, algo_out_ptr);
}

uint8_t ch_get_algo_state(ch_dev_t *dev_ptr, void *algo_state_ptr) {

	return ch_common_get_algo_state(dev_ptr, algo_state_ptr);
}

#endif  // INCLUDE_SHASTA_SUPPORT

uint32_t ch_get_cpu_frequency(ch_dev_t *dev_ptr) {

	return dev_ptr->cpu_frequency;
}

uint8_t ch_set_rtc(ch_dev_t *dev_ptr, ch_rtc_src_t rtc_source, uint16_t rtc_freq) {

	return ch_common_set_rtc(dev_ptr, rtc_source, rtc_freq);
}

uint16_t ch_get_rtc_frequency(ch_dev_t *dev_ptr) {

	return dev_ptr->rtc_frequency;
}

uint8_t ch_set_pmut_clock(ch_dev_t *dev_ptr, ch_pmut_clk_cfg_t clock_cfg) {

	return ch_common_set_pmut_clock(dev_ptr, clock_cfg);
}

ch_pmut_clk_cfg_t ch_get_pmut_clock(ch_dev_t *dev_ptr) {

	return dev_ptr->pmut_clk_cfg;
}

void ch_group_set_pmut_clock_freq(ch_group_t *grp_ptr, uint32_t pmut_clock_freq) {

	grp_ptr->pmut_clock_freq = pmut_clock_freq;
}

uint32_t ch_group_get_pmut_clock_freq(ch_group_t *grp_ptr) {

	return grp_ptr->pmut_clock_freq;
}

uint8_t ch_get_num_targets(ch_dev_t *dev_ptr) {

#ifdef INCLUDE_ALGO_RANGEFINDER
	return ch_common_get_num_targets(dev_ptr);
#else
	(void)dev_ptr;
	return 0;
#endif
}

uint32_t ch_get_target_range(ch_dev_t *dev_ptr, uint8_t target_num, ch_range_t range_type) {

#ifdef INCLUDE_ALGO_RANGEFINDER
	return ch_common_get_target_range(dev_ptr, target_num, range_type);
#else
	(void)dev_ptr;
	(void)target_num;
	(void)range_type;
	return 0;
#endif
}

uint16_t ch_get_target_amplitude(ch_dev_t *dev_ptr, uint8_t target_num) {

#ifdef INCLUDE_ALGO_RANGEFINDER
	return ch_common_get_target_amplitude(dev_ptr, target_num);
#else
	(void)dev_ptr;
	(void)target_num;
	return 0;
#endif
}

uint32_t ch_get_target_tof_us(ch_dev_t *dev_ptr, uint8_t target_num) {

#ifdef INCLUDE_ALGO_RANGEFINDER
	return ch_common_get_target_tof_us(dev_ptr, target_num);
#else
	(void)dev_ptr;
	(void)target_num;
	return 0;
#endif
}

uint16_t ch_get_num_output_samples(ch_dev_t *dev_ptr) {

	return ch_common_get_num_output_samples(dev_ptr);
}

uint32_t ch_usec_to_cycles(ch_dev_t *dev_ptr, uint32_t num_usec) {

	return ch_common_usec_to_cycles(dev_ptr, num_usec);
}

uint32_t ch_cycles_to_usec(ch_dev_t *dev_ptr, uint32_t num_cycles) {

	return ch_common_cycles_to_usec(dev_ptr, num_cycles);
}

uint32_t ch_samples_to_cycles(uint16_t num_samples, ch_odr_t odr) {

	return ch_common_samples_to_cycles(num_samples, odr);
}

uint16_t ch_cycles_to_samples(uint32_t num_cycles, ch_odr_t odr) {

	return ch_common_cycles_to_samples(num_cycles, odr);
}

uint32_t ch_usec_to_ticks(ch_dev_t *dev_ptr, uint32_t num_usec) {

	return ch_common_usec_to_ticks(dev_ptr, num_usec);
}

uint32_t ch_ticks_to_usec(ch_dev_t *dev_ptr, uint32_t num_ticks) {

	return ch_common_ticks_to_usec(dev_ptr, num_ticks);
}

uint8_t ch_watchdog_enable(ch_dev_t *dev_ptr) {

	return ch_common_watchdog_enable(dev_ptr);
}

uint8_t ch_watchdog_disable(ch_dev_t *dev_ptr) {

	return ch_common_watchdog_disable(dev_ptr);
}

uint8_t ch_check_reset_state(ch_dev_t *dev_ptr, ch_sensor_reset_state_t *reset_state_ptr) {
	uint8_t rst_st;
	uint8_t ret_val = 0;

	ret_val = chdrv_check_reset_state(dev_ptr, &rst_st);

	if (rst_st == 1) {
		*reset_state_ptr = CH_RESET_STATE_ACTIVE;
	} else {
		*reset_state_ptr = CH_RESET_STATE_OK;
	}

	return ret_val;
}
