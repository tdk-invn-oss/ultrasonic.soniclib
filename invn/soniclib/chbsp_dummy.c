/*!
 * \file chbsp_dummy.c
 *
 * \brief Dummy implementations of optional board support package IO functions allowing
 * platforms to selectively support only needed functionality.  These are placeholder
 * routines that will satisfy references from other code to avoid link errors, but they
 * do not peform any actual operations.
 *
 * See chirp_bsp.h for descriptions of all board support package interfaces, including
 * details on these optional functions.
 */

/*
 * Copyright (c) 2017-2021 Chirp Microsystems.  All rights reserved.
 */
#include <invn/soniclib/chirp_bsp.h>

/* Functions supporting debugging */

__attribute__((weak)) void chbsp_debug_toggle(uint8_t __attribute__((unused)) dbg_pin_num) {
}

__attribute__((weak)) void chbsp_debug_on(uint8_t __attribute__((unused)) dbg_pin_num) {
}

__attribute__((weak)) void chbsp_debug_off(uint8_t __attribute__((unused)) dbg_pin_num) {
}

__attribute__((weak)) void chbsp_print_str(char *str) {
	(void)(str);
}

__attribute__((weak)) uint32_t chbsp_timestamp_ms() {
	return 0;
}

__attribute__((weak)) void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int1_clear(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int1_set(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_group_int1_clear(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int1_set(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int2_interrupt_enable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int2_interrupt_enable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int2_interrupt_disable(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int2_interrupt_disable(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_set_int2_dir_out(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_set_int2_dir_in(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_set_int2_dir_out(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_group_set_int2_dir_in(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_int2_clear(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_int2_set(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_group_int2_set(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

__attribute__((weak)) void chbsp_group_int2_clear(ch_group_t *grp_ptr) {
	(void)(grp_ptr);
}

/* Functions supporting non-blocking operation */

__attribute__((weak)) int chbsp_i2c_mem_write_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data,
                                                 uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data,
                                                uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) void chbsp_external_irq_handler(chdrv_transaction_t *trans) {
	(void)(trans);
}

__attribute__((weak)) int chbsp_spi_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data,
                                                uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(mem_addr);
	(void)(data);
	(void)(num_bytes);
	return 1;
}

__attribute__((weak)) void chbsp_spi_cs_on(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) void chbsp_spi_cs_off(ch_dev_t *dev_ptr) {
	(void)(dev_ptr);
}

__attribute__((weak)) int chbsp_spi_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(data);
	(void)(num_bytes);
	return 0;
}

__attribute__((weak)) int chbsp_spi_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	(void)(dev_ptr);
	(void)(data);
	(void)(num_bytes);
	return 0;
}
