/*! \file ch101_gprmt.c
 *
 * \brief Chirp CH201 General Purpose Rangefinding Multi-Threshold firmware interface
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
#include "ch101_gprmt.h"
#include <invn/soniclib/details/ch_common.h>

uint8_t ch101_gprmt_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index) {
	
	(void)grp_ptr;

	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->bus_index = bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text 					= ch101_gprmt_fw_text;
	dev_ptr->fw_text_size 				= ch101_gprmt_text_size;
	dev_ptr->fw_vec 					= ch101_gprmt_fw_vec;
	dev_ptr->fw_vec_size 				= ch101_gprmt_vec_size;
	dev_ptr->fw_version_string			= ch101_gprmt_version;
	dev_ptr->ram_init 					= get_ram_ch101_gprmt_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_gprmt_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_gprmt_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          	= ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         	= ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  	= ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    	= ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range 	= NULL;								// not supported
	dev_ptr->api_funcs.set_rx_holdoff   	= ch_common_set_rx_holdoff;
	dev_ptr->api_funcs.get_rx_holdoff   	= ch_common_get_rx_holdoff;
	dev_ptr->api_funcs.get_range        	= ch_common_get_range;
	dev_ptr->api_funcs.get_amplitude    	= ch_common_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      	= ch_common_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data  	= ch_common_get_amplitude_data;
	dev_ptr->api_funcs.samples_to_mm    	= ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    	= ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   	= ch_common_set_thresholds;
	dev_ptr->api_funcs.get_thresholds   	= ch_common_get_thresholds;
	dev_ptr->api_funcs.set_sample_window 	= ch_common_set_sample_window;
	dev_ptr->api_funcs.get_amplitude_avg 	= ch_common_get_amplitude_avg;
	dev_ptr->api_funcs.set_rx_low_gain		= ch_common_set_rx_low_gain;
	dev_ptr->api_funcs.get_rx_low_gain		= ch_common_get_rx_low_gain;
	dev_ptr->api_funcs.set_tx_length		= NULL;								// not supported
	dev_ptr->api_funcs.get_tx_length		= ch_common_get_tx_length;

	/* Init max sample count */
	dev_ptr->max_samples = CH101_GPRMT_MAX_SAMPLES;
	
	/* Set max number of thresholds */
	dev_ptr->max_num_thresholds = CHX01_COMMON_NUM_THRESHOLDS;
	
	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	return 0;
}