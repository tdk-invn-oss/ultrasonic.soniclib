//
// InvenSense Firmware Header Generator v2.5 (Python 3.10.6)
//
// File generated at 2022-10-18 14:34:01.865516 by jenkins
// Script input parameters:
//   - Input file:                 shasta-init.hex
//   - Output file:                icu_init_fw.c
//   - Part number:                generic
//   - Program size:               6144
//   - DMEM start address:         0x1000
//   - PMEM start address:         0xe800
//   - Firmware name:              init
//   - Firmware name (sanitized):  init
//   - Firmware git version:       1.3.5-rc.1
//   - Firmware git sha1:          3fb3feadd0f47c07037cd17bf1d996c1a5d9816d
//
// Copyright (c) 2022, InvenSense. All rights reserved.
//

#include <stdint.h>
#include <invn/soniclib/details/icu.h>
#include "icu_init.h"

const char * icu_init_version = "init_1.3.5-rc.1";
const char * icu_init_gitsha1 = "3fb3feadd0f47c07037cd17bf1d996c1a5d9816d";

#define RAM_INIT_ADDRESS 4096
#define RAM_INIT_WRITE_SIZE   84

uint16_t get_icu_init_fw_ram_init_addr(void) { return (uint16_t)RAM_INIT_ADDRESS;}
uint16_t get_icu_init_fw_ram_init_size(void) { return (uint16_t)RAM_INIT_WRITE_SIZE;}

const unsigned char ram_icu_init_init[RAM_INIT_WRITE_SIZE] = {
0x40, 0x12, 0x54, 0x10, 0x14, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x50, 0x00, 0xC8, 0x00, 0x03, 0x00, 0x00, 0x00, 0x90, 0x01, 
0x00, 0x00, 0x00, 0x40, 0x00, 0x20, 0xE4, 0x12, 0xFB, 0x09, 0x11, 0x05, 0x8B, 0x02, 0x46, 0x01, 
0xA3, 0x00, 0x51, 0x00, 0x29, 0x00, 0x14, 0x00, 0x0A, 0x00, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 
0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x10, 0xFF, 0xFF, 0x01, 0x00, 0xE0, 0x01, 0x21, 0x81, 0x00, 0x04, 0x22, 0x3C, 0x20, 0x00, 
0x2A, 0x43, 0x60, 0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, };

const unsigned char * get_ram_icu_init_init_ptr(void) { return &ram_icu_init_init[0];}

#define	ICU_INIT_TEXT_SIZE	4128
#define	ICU_INIT_VEC_SIZE	32

const uint16_t  icu_init_text_size	= ICU_INIT_TEXT_SIZE;
const uint16_t  icu_init_vec_size	= ICU_INIT_VEC_SIZE;

const unsigned char icu_init_fw_text[ICU_INIT_TEXT_SIZE] = {
0x31, 0x40, 0x00, 0x20, 0x3c, 0x40, 0x54, 0x10, 0x0d, 0x43, 0x3e, 0x40, 0xec, 0x01, 0xb0, 0x12, 
0x0e, 0xf8, 0x3c, 0x40, 0x3a, 0x10, 0x3d, 0x40, 0x3a, 0x10, 0x0d, 0x9c, 0x04, 0x24, 0x3e, 0x40, 
0x1a, 0x00, 0xb0, 0x12, 0xd2, 0xf7, 0x0c, 0x43, 0xb0, 0x12, 0xb8, 0xf5, 0x0a, 0x12, 0x09, 0x12, 
0x08, 0x12, 0x07, 0x12, 0x18, 0x42, 0x3a, 0x12, 0x08, 0x93, 0x8d, 0x24, 0x1c, 0x42, 0x34, 0x12, 
0x0c, 0x5c, 0x0c, 0x5c, 0x1c, 0x52, 0x30, 0x12, 0xbc, 0xf0, 0xff, 0xf8, 0x00, 0x00, 0x1a, 0x42, 
0x34, 0x12, 0x0a, 0x5a, 0x0a, 0x5a, 0x1a, 0x52, 0x30, 0x12, 0x29, 0x4a, 0x1c, 0x42, 0x3a, 0x12, 
0x7d, 0x42, 0xb0, 0x12, 0xa2, 0xf7, 0x3c, 0xf0, 0x00, 0x07, 0x0c, 0xd9, 0x8a, 0x4c, 0x00, 0x00, 
0xb2, 0x53, 0x3a, 0x12, 0x48, 0x43, 0x0c, 0x48, 0x30, 0x40, 0x94, 0xf7, 0x0d, 0x4c, 0x3d, 0x53, 
0x82, 0x4d, 0x32, 0x12, 0x0c, 0x5c, 0x1a, 0x4c, 0x1e, 0x12, 0x5c, 0x4a, 0x05, 0x00, 0x1c, 0x52, 
0x34, 0x12, 0x0c, 0x5c, 0x0c, 0x5c, 0x1c, 0x52, 0x30, 0x12, 0xbc, 0xf0, 0xff, 0x0f, 0x00, 0x00, 
0x59, 0x4a, 0x05, 0x00, 0x19, 0x52, 0x34, 0x12, 0x09, 0x59, 0x09, 0x59, 0x19, 0x52, 0x30, 0x12, 
0x27, 0x49, 0x5c, 0x4a, 0x04, 0x00, 0x7d, 0x40, 0x0c, 0x00, 0xb0, 0x12, 0xa2, 0xf7, 0x0c, 0xd7, 
0x3c, 0xd0, 0x20, 0x00, 0x89, 0x4c, 0x00, 0x00, 0x1d, 0x4a, 0x02, 0x00, 0x0d, 0x93, 0x28, 0x20, 
0x8a, 0x93, 0x00, 0x00, 0x31, 0x20, 0x1a, 0x42, 0x34, 0x12, 0x1d, 0x42, 0x34, 0x12, 0x1c, 0x42, 
0x36, 0x12, 0x0c, 0x5d, 0x0a, 0x9c, 0x04, 0x28, 0xb2, 0x40, 0x03, 0x00, 0x3a, 0x12, 0xc3, 0x3f, 
0x6c, 0x42, 0x0d, 0x9a, 0x01, 0x24, 0x5c, 0x43, 0x09, 0x4a, 0x09, 0x59, 0x09, 0x59, 0x1d, 0x42, 
0x30, 0x12, 0x0d, 0x59, 0xbd, 0xf0, 0xff, 0xf8, 0x00, 0x00, 0x19, 0x52, 0x30, 0x12, 0x27, 0x49, 
0x7d, 0x42, 0xb0, 0x12, 0xa2, 0xf7, 0x0c, 0xd7, 0x89, 0x4c, 0x00, 0x00, 0x1a, 0x53, 0xdd, 0x3f, 
0x1c, 0x42, 0x32, 0x12, 0x6e, 0x42, 0x0e, 0x9c, 0x07, 0x28, 0x0e, 0x4c, 0x1e, 0x53, 0x82, 0x4e, 
0x32, 0x12, 0x0c, 0x5c, 0x8c, 0x4d, 0x20, 0x12, 0x2d, 0x4a, 0x0d, 0x93, 0x0c, 0x24, 0x1c, 0x42, 
0x32, 0x12, 0x6e, 0x42, 0x0e, 0x9c, 0x07, 0x28, 0x0e, 0x4c, 0x1e, 0x53, 0x82, 0x4e, 0x32, 0x12, 
0x0c, 0x5c, 0x8c, 0x4d, 0x20, 0x12, 0x1c, 0x42, 0x32, 0x12, 0x0c, 0x93, 0x8f, 0x23, 0x82, 0x93, 
0x3e, 0x12, 0x08, 0x24, 0x82, 0x43, 0x3e, 0x12, 0x92, 0x43, 0x32, 0x12, 0x92, 0x42, 0x2c, 0x12, 
0x20, 0x12, 0xf1, 0x3f, 0x78, 0x40, 0x03, 0x00, 0x7e, 0x3f, 0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 
0x0f, 0x4c, 0x4a, 0x4d, 0x49, 0x4e, 0x7c, 0x40, 0x05, 0x00, 0x4c, 0x9a, 0x51, 0x28, 0x1a, 0x93, 
0x52, 0x24, 0x1c, 0x42, 0x38, 0x12, 0x0d, 0x4c, 0x1d, 0x53, 0x82, 0x4d, 0x38, 0x12, 0x0d, 0x4c, 
0x0d, 0x5d, 0x0d, 0x5c, 0x0d, 0x5d, 0x0c, 0x4d, 0x3c, 0x50, 0xa0, 0x10, 0x8d, 0x43, 0xa0, 0x10, 
0x8c, 0x43, 0x02, 0x00, 0x8d, 0x43, 0xa4, 0x10, 0xdc, 0x4f, 0x04, 0x00, 0x04, 0x00, 0x5e, 0x4f, 
0x05, 0x00, 0x5e, 0x53, 0xcc, 0x4e, 0x05, 0x00, 0x8f, 0x4c, 0x00, 0x00, 0x1b, 0x42, 0x38, 0x12, 
0x0d, 0x4b, 0x1d, 0x53, 0x82, 0x4d, 0x38, 0x12, 0x0d, 0x4b, 0x0d, 0x5d, 0x0e, 0x4d, 0x0e, 0x5b, 
0x0e, 0x5e, 0x08, 0x4e, 0x38, 0x50, 0xa0, 0x10, 0x8e, 0x43, 0xa0, 0x10, 0x88, 0x43, 0x02, 0x00, 
0x8e, 0x43, 0xa4, 0x10, 0x5e, 0x4f, 0x04, 0x00, 0x09, 0x93, 0x18, 0x20, 0x7e, 0x53, 0x7e, 0xf0, 
0x0f, 0x00, 0x0d, 0x5b, 0x0d, 0x5d, 0xcd, 0x4e, 0xa4, 0x10, 0x5e, 0x4f, 0x05, 0x00, 0x5e, 0x53, 
0xcd, 0x4e, 0xa5, 0x10, 0x8f, 0x48, 0x02, 0x00, 0x4d, 0x4a, 0x7d, 0x53, 0x4a, 0x4d, 0x4e, 0x49, 
0xb0, 0x12, 0x7a, 0xe9, 0x0c, 0x93, 0x05, 0x20, 0x0f, 0x48, 0xad, 0x3f, 0x5e, 0x53, 0xe7, 0x3f, 
0x6c, 0x43, 0x30, 0x40, 0x96, 0xf7, 0x4c, 0x43, 0xfc, 0x3f, 0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 
0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x21, 0x83, 0x09, 0x4c, 0x0a, 0x4d, 0x46, 0x4e, 
0x81, 0x4f, 0x00, 0x00, 0x4c, 0x43, 0x0c, 0x9d, 0x26, 0x34, 0x4d, 0x43, 0x0d, 0x89, 0x09, 0x4d, 
0x35, 0x40, 0x00, 0xc0, 0x0a, 0x11, 0x09, 0x11, 0x37, 0x40, 0x18, 0x10, 0x48, 0x43, 0x0c, 0x4a, 
0x0d, 0x48, 0xb0, 0x12, 0xac, 0xf7, 0x04, 0x4c, 0x0c, 0x49, 0x0d, 0x48, 0xb0, 0x12, 0xac, 0xf7, 
0x3d, 0x47, 0x4e, 0x43, 0x0e, 0x99, 0x15, 0x34, 0x0a, 0x5c, 0x09, 0x84, 0x05, 0x8d, 0x18, 0x53, 
0x48, 0x96, 0xed, 0x2b, 0x4c, 0x43, 0x0c, 0x85, 0x2e, 0x41, 0x8e, 0x4c, 0x00, 0x00, 0x0c, 0x4a, 
0x21, 0x53, 0x30, 0x40, 0x8e, 0xf7, 0x4e, 0x43, 0x0e, 0x8d, 0x0a, 0x4e, 0x35, 0x40, 0x00, 0x40, 
0xd9, 0x3f, 0x0a, 0x8c, 0x09, 0x54, 0x05, 0x5d, 0xea, 0x3f, 0xa2, 0xb3, 0x94, 0x01, 0xfd, 0x27, 
0x30, 0x41, 0x1c, 0x83, 0xfe, 0x23, 0x30, 0x41, 0x82, 0xdc, 0x44, 0x12, 0x1c, 0x42, 0x40, 0x10, 
0xdc, 0x43, 0x03, 0x00, 0xc2, 0x93, 0x9a, 0x10, 0x02, 0x24, 0xcc, 0x43, 0x03, 0x00, 0x30, 0x41, 
0x5e, 0x42, 0x56, 0x12, 0x4d, 0x43, 0x0e, 0xbc, 0x06, 0x24, 0x92, 0xb3, 0x44, 0x12, 0x05, 0x20, 
0x5d, 0x43, 0x1d, 0xc2, 0x98, 0x10, 0x0c, 0x4d, 0x30, 0x41, 0x4d, 0x43, 0xfc, 0x3f, 0x3d, 0x40, 
0xa0, 0x01, 0x9d, 0x42, 0x9c, 0x10, 0x06, 0x00, 0x82, 0x4c, 0x88, 0x10, 0x9d, 0x43, 0x08, 0x00, 
0x92, 0xd3, 0x98, 0x10, 0x30, 0x41, 0x1c, 0x42, 0x20, 0x01, 0x7c, 0xf0, 0x80, 0x00, 0x3c, 0xd0, 
0x18, 0x5a, 0x82, 0x4c, 0x20, 0x01, 0x3c, 0x40, 0xa0, 0x01, 0xbc, 0x40, 0x10, 0x14, 0x0a, 0x00, 
0xbc, 0x40, 0x10, 0x14, 0x0e, 0x00, 0xac, 0x43, 0x08, 0x00, 0x8c, 0x43, 0x08, 0x00, 0x30, 0x41, 
0x1c, 0x42, 0xa0, 0x01, 0x3c, 0xf0, 0xff, 0xf8, 0x3c, 0xd0, 0x00, 0x06, 0x82, 0x4c, 0xa0, 0x01, 
0xf2, 0x40, 0x06, 0x00, 0x9e, 0x10, 0xb2, 0x40, 0x42, 0x10, 0x9c, 0x10, 0xb0, 0x12, 0x16, 0xeb, 
0xb2, 0x90, 0x03, 0x00, 0x52, 0x10, 0x03, 0x20, 0xb2, 0x40, 0x2a, 0x55, 0x4a, 0x10, 0x30, 0x41, 
0x0a, 0x12, 0x09, 0x12, 0x5a, 0x42, 0x55, 0x12, 0x5c, 0x43, 0x4c, 0x9a, 0x01, 0x2c, 0x4a, 0x43, 
0x0c, 0x4a, 0x7d, 0x40, 0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 0x5c, 0x4c, 0xda, 0x12, 0x7c, 0xf0, 
0x07, 0x00, 0xc2, 0x4c, 0x9e, 0x10, 0x0c, 0x4a, 0x7d, 0x40, 0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 
0x5c, 0x4c, 0xda, 0x12, 0x19, 0x42, 0xa0, 0x01, 0x7c, 0xf0, 0x07, 0x00, 0x7d, 0x42, 0xb0, 0x12, 
0xa2, 0xf7, 0x39, 0xf0, 0xff, 0xf8, 0x0c, 0xd9, 0x82, 0x4c, 0xa0, 0x01, 0x0c, 0x4a, 0x7d, 0x40, 
0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 0x3c, 0x50, 0x58, 0x12, 0x82, 0x4c, 0x9c, 0x10, 0xb0, 0x12, 
0x16, 0xeb, 0x30, 0x40, 0x98, 0xf7, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 
0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x3c, 0x40, 0x00, 0x04, 0xb0, 0x12, 
0xc8, 0xea, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 
0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x6c, 0x43, 
0xb0, 0x12, 0xe0, 0xea, 0x0c, 0x93, 0x03, 0x24, 0x5c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 0xb1, 0xc0, 
0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 
0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x5c, 0x43, 0xb0, 0x12, 0xe0, 0xea, 
0x0c, 0x93, 0x03, 0x24, 0x5c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 
0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0d, 0x12, 0x0c, 0x12, 
0x3d, 0x40, 0xd8, 0x01, 0x3c, 0x40, 0x40, 0x12, 0x9c, 0x4d, 0x02, 0x00, 0xc2, 0x01, 0xed, 0x43, 
0x00, 0x00, 0xbc, 0xd0, 0x20, 0x00, 0x04, 0x00, 0xb1, 0xc0, 0xf0, 0x00, 0x04, 0x00, 0x3c, 0x41, 
0x3d, 0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0xb2, 0xf0, 
0xfd, 0xfe, 0x98, 0x10, 0xf2, 0xd2, 0x58, 0x00, 0x92, 0xb3, 0xd4, 0x01, 0xfd, 0x27, 0x1c, 0x42, 
0x96, 0x10, 0x0c, 0x93, 0x14, 0x24, 0x82, 0x43, 0x96, 0x10, 0x92, 0x42, 0xd2, 0x01, 0x06, 0x14, 
0x92, 0xb3, 0x98, 0x10, 0x03, 0x20, 0x6c, 0x42, 0xb0, 0x12, 0xc8, 0xea, 0xb1, 0xc0, 0xf0, 0x00, 
0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x92, 0x42, 
0xd2, 0x01, 0x04, 0x14, 0x92, 0xb3, 0x98, 0x10, 0xf1, 0x23, 0x5c, 0x42, 0x51, 0x12, 0x8c, 0x11, 
0x0c, 0x93, 0x0d, 0x24, 0x5c, 0x42, 0x51, 0x12, 0x8c, 0x11, 0x1d, 0x42, 0xf0, 0x01, 0x0c, 0x5d, 
0x3c, 0xf0, 0xff, 0x01, 0x3d, 0xf0, 0x00, 0xfe, 0x0c, 0xdd, 0x82, 0x4c, 0xf0, 0x01, 0x6c, 0x43, 
0xdb, 0x3f, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x1a, 0x42, 
0x40, 0x10, 0xca, 0x43, 0x03, 0x00, 0x7c, 0x40, 0x4c, 0x00, 0xb0, 0x12, 0xc2, 0xea, 0xea, 0xd2, 
0x00, 0x00, 0x1c, 0x42, 0x48, 0x12, 0x0c, 0x93, 0x0f, 0x20, 0xb2, 0xb2, 0x04, 0x02, 0x02, 0x24, 
0x82, 0x43, 0x44, 0x12, 0xb1, 0xc0, 0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 
0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x92, 0xb3, 0x48, 0x12, 0x3b, 0x24, 0x92, 0xc3, 
0x48, 0x12, 0x5c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 0xb2, 0xb0, 0x00, 0x04, 0x48, 0x12, 0xf2, 0x24, 
0xb2, 0xf0, 0xff, 0xfb, 0x48, 0x12, 0xc2, 0x93, 0x8c, 0x10, 0x04, 0x24, 0x5c, 0x42, 0xbe, 0x13, 
0x0c, 0x93, 0x1a, 0x20, 0x5c, 0x42, 0xbe, 0x13, 0xb2, 0xf0, 0xff, 0xf9, 0xf0, 0x01, 0xf2, 0xf0, 
0xfc, 0xff, 0xf2, 0x01, 0x3c, 0xb0, 0x07, 0x00, 0x0f, 0x24, 0xe2, 0x43, 0xc0, 0x01, 0xe2, 0x43, 
0xc1, 0x01, 0xb2, 0x40, 0xa5, 0x00, 0xf4, 0x01, 0xb2, 0xd0, 0x00, 0x04, 0xf0, 0x01, 0xb2, 0x40, 
0xa5, 0x00, 0xf4, 0x01, 0xe2, 0xd3, 0xf2, 0x01, 0x82, 0x43, 0x8a, 0x10, 0x5d, 0x42, 0xbe, 0x13, 
0x5c, 0x42, 0xbf, 0x13, 0xc2, 0x4d, 0xc0, 0x01, 0xc2, 0x4c, 0xc1, 0x01, 0xd2, 0x42, 0xbe, 0x13, 
0x8c, 0x10, 0xaf, 0x3f, 0xb2, 0xb0, 0x00, 0x02, 0x48, 0x12, 0x06, 0x24, 0xb2, 0xf0, 0xff, 0xfd, 
0x48, 0x12, 0xb0, 0x12, 0x70, 0xeb, 0xc0, 0x3f, 0xb2, 0xb0, 0x00, 0x08, 0x48, 0x12, 0x41, 0x24, 
0xb2, 0xf0, 0xff, 0xf7, 0x48, 0x12, 0xd2, 0xb3, 0x52, 0x12, 0x33, 0x20, 0x3c, 0x40, 0xe0, 0x01, 
0x82, 0x4c, 0x40, 0x10, 0x5c, 0x42, 0x52, 0x12, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 
0x5c, 0xf3, 0xc2, 0x4c, 0x9a, 0x10, 0xe2, 0xb2, 0x56, 0x12, 0x26, 0x24, 0x92, 0xb3, 0x94, 0x01, 
0xfd, 0x27, 0xb2, 0x40, 0x0a, 0x00, 0x90, 0x01, 0x3a, 0x40, 0xba, 0xea, 0x8a, 0x12, 0xb2, 0x40, 
0x03, 0x00, 0x92, 0x01, 0x8a, 0x12, 0xb2, 0x40, 0x07, 0x00, 0x92, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 
0xd2, 0xc3, 0xe4, 0x01, 0xd2, 0xb3, 0x56, 0x12, 0x02, 0x24, 0xd2, 0xd3, 0xe0, 0x01, 0xe2, 0xb3, 
0x56, 0x12, 0x02, 0x24, 0xd2, 0xd3, 0xe4, 0x01, 0x5c, 0x42, 0x56, 0x12, 0x0c, 0x93, 0x84, 0x27, 
0xc0, 0x3f, 0x3c, 0x40, 0xe4, 0x01, 0xcc, 0x3f, 0xb0, 0x12, 0xba, 0xea, 0x82, 0x43, 0x92, 0x01, 
0xe5, 0x3f, 0xa2, 0xb3, 0x48, 0x12, 0x0f, 0x24, 0xa2, 0xc3, 0x48, 0x12, 0x1c, 0x42, 0x4e, 0x12, 
0xf2, 0xd2, 0x58, 0x00, 0xa2, 0xd3, 0x98, 0x10, 0x3c, 0xf0, 0xff, 0x0f, 0x3c, 0xd0, 0x00, 0x10, 
0x82, 0x4c, 0xd0, 0x01, 0x69, 0x3f, 0xa2, 0xb2, 0x48, 0x12, 0x14, 0x24, 0xa2, 0xc2, 0x48, 0x12, 
0x1c, 0x42, 0x4e, 0x12, 0xf2, 0xc2, 0x58, 0x00, 0xb2, 0xd0, 0x00, 0x01, 0x98, 0x10, 0x92, 0x43, 
0x96, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0xe4, 0x3f, 0xb2, 0xb0, 0x20, 0x00, 0x48, 0x12, 0x08, 0x24, 0xb2, 0xf0, 0xdf, 0xff, 
0x48, 0x12, 0xc2, 0x43, 0xd9, 0x01, 0xd2, 0x43, 0xd8, 0x01, 0x46, 0x3f, 0xb2, 0xb0, 0x00, 0x10, 
0x48, 0x12, 0x2e, 0x24, 0xb2, 0xf0, 0xff, 0xef, 0x48, 0x12, 0x5c, 0x42, 0x4b, 0x12, 0x7c, 0xd0, 
0x40, 0x00, 0xc2, 0x4c, 0xf3, 0x01, 0x92, 0x42, 0x4c, 0x12, 0xf0, 0x01, 0xd2, 0x42, 0x4a, 0x12, 
0xf2, 0x01, 0x5c, 0x42, 0x50, 0x12, 0x7c, 0xf0, 0xf0, 0xff, 0x7c, 0x90, 0x50, 0x00, 0x08, 0x20, 
0xe2, 0xb2, 0x50, 0x12, 0x0e, 0x24, 0xb2, 0x40, 0x18, 0x5a, 0x20, 0x01, 0xd2, 0xd3, 0x00, 0x00, 
0xd2, 0xb3, 0x50, 0x12, 0x0a, 0x24, 0xa2, 0xd2, 0x98, 0x10, 0xf2, 0xf0, 0x0f, 0x00, 0x50, 0x12, 
0x1b, 0x3f, 0xb2, 0x40, 0x80, 0x5a, 0x20, 0x01, 0xf3, 0x3f, 0xa2, 0xc2, 0x98, 0x10, 0xf5, 0x3f, 
0xb2, 0xb0, 0x00, 0x20, 0x48, 0x12, 0x10, 0x27, 0xb2, 0xf0, 0xff, 0xdf, 0x48, 0x12, 0xc2, 0x43, 
0x3e, 0x10, 0x0a, 0x3f, 0xb2, 0xb0, 0x80, 0x00, 0x48, 0x12, 0x08, 0x24, 0xb2, 0xf0, 0x7f, 0xff, 
0x48, 0x12, 0x7c, 0x40, 0x80, 0x00, 0xb0, 0x12, 0xc8, 0xea, 0xe3, 0x3e, 0xb2, 0xb2, 0x48, 0x12, 
0x38, 0x24, 0xb2, 0xc2, 0x48, 0x12, 0xb0, 0x12, 0x40, 0xeb, 0x1c, 0x42, 0x52, 0x10, 0x2c, 0x93, 
0x28, 0x24, 0x3c, 0x90, 0x03, 0x00, 0x29, 0x20, 0xb2, 0x40, 0x10, 0x10, 0x3a, 0x10, 0xb2, 0x40, 
0x88, 0x13, 0x94, 0x10, 0x1c, 0x42, 0x3a, 0x10, 0x2d, 0x4c, 0x82, 0x4d, 0x92, 0x10, 0x82, 0x43, 
0x90, 0x10, 0x82, 0x43, 0x8e, 0x10, 0x5c, 0x4c, 0x04, 0x00, 0xc2, 0x4c, 0x8d, 0x10, 0x1a, 0x42, 
0xf0, 0x01, 0x3a, 0xf0, 0x00, 0x3e, 0x3d, 0xf0, 0xff, 0x01, 0x0a, 0xdd, 0x7d, 0x40, 0x0e, 0x00, 
0xb0, 0x12, 0xa2, 0xf7, 0x0a, 0xdc, 0x82, 0x4a, 0xf0, 0x01, 0x6c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 
0xb0, 0x3e, 0xb2, 0x40, 0x0a, 0x10, 0x3a, 0x10, 0xda, 0x3f, 0xb2, 0x40, 0x04, 0x10, 0x3a, 0x10, 
0xd6, 0x3f, 0xb2, 0xb0, 0x10, 0x00, 0x48, 0x12, 0xa4, 0x26, 0xb2, 0x43, 0x3c, 0x10, 0xb2, 0xf0, 
0xef, 0xff, 0x48, 0x12, 0x82, 0x43, 0x6a, 0x13, 0xb0, 0x12, 0x40, 0xeb, 0x6c, 0x42, 0xe6, 0x3f, 
0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x5c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 
0x5c, 0x42, 0x55, 0x12, 0x5d, 0x43, 0x4d, 0x9c, 0x01, 0x2c, 0x4c, 0x43, 0x7d, 0x40, 0x84, 0x00, 
0xb0, 0x12, 0xbe, 0xf7, 0x1c, 0x4c, 0xd8, 0x12, 0x92, 0xb3, 0x94, 0x01, 0xfd, 0x27, 0x82, 0x4c, 
0x90, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 
0x3f, 0x41, 0x00, 0x13, 0xb2, 0x40, 0x0f, 0x00, 0xa4, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 
0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x09, 0x12, 
0x08, 0x12, 0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x31, 0x80, 0x2c, 0x00, 0x1c, 0x42, 
0xb0, 0x01, 0x3d, 0x40, 0x10, 0x14, 0x3d, 0xf0, 0xff, 0x0f, 0x0c, 0x8d, 0x82, 0x4c, 0x0e, 0x14, 
0xd2, 0x42, 0x9e, 0x10, 0x09, 0x14, 0x92, 0xc3, 0x98, 0x10, 0x5c, 0x42, 0x55, 0x12, 0xc2, 0x4c, 
0x08, 0x14, 0x1a, 0x42, 0x88, 0x10, 0x1a, 0x93, 0x3d, 0x20, 0xc2, 0x93, 0x3e, 0x10, 0x02, 0x24, 
0x3a, 0x40, 0x00, 0x20, 0x5e, 0x42, 0x55, 0x12, 0x5d, 0x42, 0x53, 0x12, 0x5e, 0x53, 0x5f, 0x42, 
0x54, 0x12, 0x4f, 0x9e, 0x05, 0x28, 0x4d, 0x9e, 0x01, 0x2c, 0x4d, 0x4e, 0x3d, 0xf0, 0xff, 0x00, 
0xc2, 0x4d, 0x55, 0x12, 0x81, 0x4c, 0x00, 0x00, 0xb0, 0x12, 0x70, 0xeb, 0x5d, 0x42, 0x51, 0x12, 
0x8d, 0x11, 0x2c, 0x41, 0x0d, 0x93, 0x0d, 0x24, 0x5d, 0x42, 0x51, 0x12, 0x8d, 0x11, 0x1e, 0x42, 
0xf0, 0x01, 0x0d, 0x5e, 0x3d, 0xf0, 0xff, 0x01, 0x3e, 0xf0, 0x00, 0xfe, 0x0d, 0xde, 0x82, 0x4d, 
0xf0, 0x01, 0x7d, 0x40, 0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 0x5c, 0x4c, 0xdb, 0x12, 0x5c, 0xf3, 
0x5d, 0x42, 0x52, 0x12, 0x6d, 0xf3, 0x4c, 0xdd, 0x4c, 0x93, 0x6f, 0x20, 0x0c, 0x4a, 0xb0, 0x12, 
0xc8, 0xea, 0x6b, 0x3c, 0x1c, 0x42, 0x88, 0x10, 0x2c, 0x93, 0x9e, 0x20, 0xb2, 0x40, 0x42, 0x10, 
0x9c, 0x10, 0xb0, 0x12, 0x16, 0xeb, 0xf2, 0x43, 0x08, 0x14, 0x0f, 0x41, 0x2f, 0x52, 0x7e, 0x42, 
0x1d, 0x42, 0x2c, 0x14, 0x1c, 0x42, 0x2e, 0x14, 0xb0, 0x12, 0x3a, 0xea, 0x1d, 0x42, 0x90, 0x10, 
0x1a, 0x42, 0x92, 0x10, 0x0d, 0x93, 0x63, 0x20, 0x1d, 0x42, 0x94, 0x10, 0x0d, 0x9c, 0x26, 0x2c, 
0x92, 0x43, 0x90, 0x10, 0x82, 0x43, 0x8e, 0x10, 0x0f, 0x41, 0x2f, 0x53, 0x7e, 0x42, 0x1d, 0x42, 
0x30, 0x14, 0x1c, 0x42, 0x32, 0x14, 0xb0, 0x12, 0x3a, 0xea, 0x1f, 0x41, 0x02, 0x00, 0x1c, 0x41, 
0x04, 0x00, 0x0e, 0x4f, 0x0e, 0x8c, 0x81, 0x4e, 0x04, 0x00, 0x1d, 0x42, 0x8e, 0x10, 0x46, 0x43, 
0x06, 0x9e, 0x48, 0x34, 0x0d, 0x93, 0x06, 0x34, 0xa2, 0x43, 0x90, 0x10, 0x0c, 0x43, 0x0c, 0x8d, 
0x0c, 0x9e, 0x16, 0x34, 0x1a, 0x53, 0x82, 0x4a, 0x92, 0x10, 0x12, 0x3c, 0x3a, 0x50, 0x0f, 0x00, 
0x82, 0x4a, 0x92, 0x10, 0x1e, 0x42, 0x3a, 0x10, 0x1c, 0x4e, 0x02, 0x00, 0x0c, 0x9a, 0x08, 0x2c, 
0x2a, 0x5e, 0x0a, 0x8c, 0x82, 0x4a, 0x92, 0x10, 0x12, 0xc3, 0x0d, 0x10, 0x82, 0x4d, 0x94, 0x10, 
0x92, 0x41, 0x04, 0x00, 0x8e, 0x10, 0x1c, 0x42, 0x90, 0x10, 0x1e, 0x42, 0xf0, 0x01, 0x3e, 0xf0, 
0x00, 0xfe, 0x1d, 0x42, 0x92, 0x10, 0x3d, 0xf0, 0xff, 0x01, 0x0e, 0xdd, 0x82, 0x4e, 0xf0, 0x01, 
0x2c, 0x93, 0x22, 0x24, 0x6c, 0x43, 0xb0, 0x12, 0xfe, 0xea, 0xb1, 0xc0, 0xf0, 0x00, 0x44, 0x00, 
0x31, 0x50, 0x2c, 0x00, 0x34, 0x41, 0x35, 0x41, 0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39, 0x41, 
0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x1d, 0x93, 
0xd7, 0x23, 0xa2, 0x3f, 0x47, 0x43, 0x07, 0x9d, 0x05, 0x34, 0xa2, 0x43, 0x90, 0x10, 0x0c, 0x8f, 
0x0d, 0x9c, 0xce, 0x37, 0x3a, 0x53, 0xb7, 0x3f, 0x1a, 0x42, 0x92, 0x10, 0x3a, 0xf0, 0xff, 0x01, 
0x1c, 0x42, 0x4c, 0x12, 0x3c, 0xf0, 0x00, 0x3e, 0x0a, 0xdc, 0x5c, 0x42, 0x8d, 0x10, 0x7d, 0x40, 
0x0e, 0x00, 0xb0, 0x12, 0xa2, 0xf7, 0x0a, 0xdc, 0x82, 0x4a, 0x4c, 0x12, 0x7c, 0x42, 0xb0, 0x12, 
0xc8, 0xea, 0x92, 0xc3, 0x98, 0x10, 0xc9, 0x3f, 0x1c, 0x42, 0x88, 0x10, 0x2c, 0x92, 0xb0, 0x21, 
0x59, 0x42, 0x55, 0x12, 0x5c, 0x43, 0x4c, 0x99, 0x01, 0x2c, 0x49, 0x43, 0x1a, 0x42, 0x6a, 0x13, 
0x0a, 0x93, 0xbc, 0x20, 0x0c, 0x49, 0x7d, 0x40, 0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 0xec, 0x42, 
0xda, 0x12, 0x0c, 0x49, 0x7d, 0x40, 0x84, 0x00, 0xb0, 0x12, 0xbe, 0xf7, 0x08, 0x4c, 0x38, 0x50, 
0x58, 0x12, 0x35, 0x40, 0x10, 0x14, 0x04, 0x41, 0x24, 0x52, 0x0b, 0x4a, 0x06, 0x4a, 0x37, 0x43, 
0x0f, 0x41, 0x2f, 0x53, 0x6e, 0x42, 0x2d, 0x45, 0x1c, 0x45, 0x02, 0x00, 0x81, 0x4b, 0x00, 0x00, 
0xb0, 0x12, 0x3a, 0xea, 0x84, 0x4c, 0x00, 0x00, 0x24, 0x53, 0x2b, 0x41, 0x0c, 0x9b, 0x02, 0x28, 
0x07, 0x46, 0x0b, 0x4c, 0x16, 0x53, 0x25, 0x52, 0x36, 0x90, 0x14, 0x00, 0xe9, 0x23, 0x37, 0x93, 
0x26, 0x24, 0x17, 0x53, 0x7d, 0x40, 0x12, 0x00, 0x0d, 0x97, 0x21, 0x28, 0x0c, 0x47, 0x0c, 0x5c, 
0x7e, 0x40, 0x2c, 0x00, 0x0e, 0x51, 0x0c, 0x5e, 0x1c, 0x4c, 0xd8, 0xff, 0x0d, 0x4c, 0x12, 0xc3, 
0x0d, 0x10, 0x12, 0xc3, 0x0d, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0x0d, 0x5c, 0x0f, 0x47, 0x0c, 0x4f, 0x0c, 0x5c, 0x66, 0x42, 0x06, 0x51, 0x0c, 0x56, 
0x2c, 0x4c, 0x0c, 0x9d, 0x0e, 0x28, 0x1f, 0x53, 0x3f, 0x90, 0x14, 0x00, 0xf4, 0x23, 0x7c, 0x40, 
0xfb, 0x00, 0xa2, 0x93, 0x52, 0x10, 0x02, 0x24, 0x7c, 0x40, 0x73, 0x00, 0x82, 0x4c, 0x6a, 0x13, 
0x32, 0x3c, 0x07, 0x9f, 0xf4, 0x2f, 0x0e, 0x4f, 0x3e, 0x53, 0x0e, 0x5e, 0x76, 0x40, 0x2c, 0x00, 
0x06, 0x51, 0x0e, 0x56, 0x15, 0x4e, 0xd8, 0xff, 0x0d, 0x8c, 0x0d, 0x5d, 0x0b, 0x45, 0x0b, 0x8c, 
0x74, 0x42, 0x46, 0x43, 0x06, 0x56, 0x36, 0xf0, 0xff, 0x00, 0x0b, 0x9d, 0x06, 0x2c, 0x0e, 0x4c, 
0x0e, 0x85, 0x0d, 0x5e, 0x56, 0x53, 0x36, 0xf0, 0xff, 0x00, 0x0d, 0x5d, 0x4e, 0x44, 0x7e, 0x53, 
0x44, 0x4e, 0x4e, 0x93, 0xef, 0x23, 0x0c, 0x4f, 0x0c, 0x87, 0x7d, 0x42, 0xb0, 0x12, 0xa2, 0xf7, 
0x0c, 0x86, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x82, 0x4c, 
0x6a, 0x13, 0x0c, 0x93, 0xc4, 0x27, 0x1c, 0x42, 0x6a, 0x13, 0x82, 0x43, 0x32, 0x12, 0x82, 0x48, 
0x30, 0x12, 0xb2, 0x40, 0x05, 0x00, 0x36, 0x12, 0x0d, 0x48, 0x4e, 0x43, 0x4f, 0x43, 0x27, 0x4d, 
0x77, 0xf0, 0x03, 0x00, 0x17, 0x93, 0x0a, 0x20, 0x16, 0x4d, 0x02, 0x00, 0x07, 0x43, 0x0e, 0x56, 
0x0f, 0x67, 0x1a, 0x53, 0x2d, 0x52, 0x3a, 0x90, 0x20, 0x00, 0xf1, 0x23, 0x82, 0x4a, 0x34, 0x12, 
0x92, 0x43, 0x3e, 0x12, 0x82, 0x43, 0x38, 0x12, 0x82, 0x43, 0x3a, 0x12, 0x0d, 0x4a, 0x3d, 0x53, 
0x77, 0x40, 0x1d, 0x00, 0x07, 0x9d, 0x2b, 0x2c, 0xb0, 0x12, 0x2c, 0xe8, 0xb0, 0x12, 0x70, 0xeb, 
0x3a, 0x40, 0x10, 0x14, 0x48, 0x43, 0x0f, 0x41, 0x2f, 0x52, 0x6e, 0x42, 0x2d, 0x4a, 0x1c, 0x4a, 
0x02, 0x00, 0xb0, 0x12, 0x3a, 0xea, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x08, 0x5c, 0x2a, 0x52, 0x3a, 0x90, 0x34, 0x14, 0xeb, 0x23, 
0x18, 0x92, 0x3c, 0x10, 0x8b, 0x28, 0xb0, 0x12, 0x2c, 0xe8, 0x4a, 0x43, 0x3c, 0x90, 0x03, 0x00, 
0xbc, 0x24, 0xb0, 0x12, 0x70, 0xeb, 0x6c, 0x42, 0xb0, 0x12, 0xfe, 0xea, 0xbd, 0x3c, 0x0a, 0x5a, 
0x0a, 0x5a, 0x0d, 0x48, 0x0d, 0x5a, 0xbd, 0x40, 0x2a, 0x32, 0x00, 0x00, 0xbd, 0x40, 0x00, 0x05, 
0x02, 0x00, 0x0d, 0x48, 0x0d, 0x5a, 0xbd, 0x40, 0x03, 0x00, 0x04, 0x00, 0x0f, 0x93, 0x03, 0x20, 
0x0e, 0x9c, 0x01, 0x2c, 0x0c, 0x4e, 0x7d, 0x40, 0x05, 0x00, 0xb0, 0x12, 0x86, 0xf7, 0x82, 0x4c, 
0x3c, 0x12, 0x08, 0x5a, 0x82, 0x43, 0xa0, 0x10, 0x82, 0x43, 0xa2, 0x10, 0xc2, 0x43, 0xa5, 0x10, 
0xb2, 0x40, 0xa0, 0x10, 0x2a, 0x12, 0x1c, 0x48, 0xfc, 0xff, 0x7d, 0x40, 0x0c, 0x00, 0xb0, 0x12, 
0xb8, 0xf7, 0x3c, 0x52, 0x7c, 0xf0, 0x0f, 0x00, 0xc2, 0x4c, 0xa4, 0x10, 0xa2, 0x43, 0x38, 0x12, 
0x82, 0x43, 0xa6, 0x10, 0x82, 0x43, 0xa8, 0x10, 0xc2, 0x43, 0xab, 0x10, 0xb2, 0x40, 0xa6, 0x10, 
0x2c, 0x12, 0xc2, 0x4c, 0xaa, 0x10, 0x3a, 0x40, 0x7a, 0xe9, 0x4e, 0x43, 0x7d, 0x40, 0x05, 0x00, 
0x3c, 0x40, 0xa0, 0x10, 0x8a, 0x12, 0x0c, 0x93, 0x8f, 0x23, 0x5e, 0x43, 0x7d, 0x40, 0x05, 0x00, 
0x1c, 0x42, 0x2c, 0x12, 0x8a, 0x12, 0x0c, 0x93, 0x87, 0x23, 0x1c, 0x42, 0x32, 0x12, 0x6d, 0x42, 
0x0d, 0x9c, 0x08, 0x28, 0x0e, 0x4c, 0x1e, 0x53, 0x82, 0x4e, 0x32, 0x12, 0x0c, 0x5c, 0x9c, 0x42, 
0x2a, 0x12, 0x20, 0x12, 0x1d, 0x42, 0x34, 0x12, 0x1c, 0x42, 0x36, 0x12, 0x1e, 0x42, 0x34, 0x12, 
0x0e, 0x5c, 0x0d, 0x9e, 0x71, 0x2f, 0x1f, 0x42, 0x30, 0x12, 0x0e, 0x4d, 0x0e, 0x5e, 0x0e, 0x5e, 
0x0c, 0x5d, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5f, 0x0f, 0x5e, 0xac, 0x4f, 0x00, 0x00, 0x9c, 0x4f, 
0x02, 0x00, 0x02, 0x00, 0x1c, 0x42, 0x30, 0x12, 0x0c, 0x5e, 0x9c, 0x43, 0x00, 0x00, 0x1e, 0x52, 
0x30, 0x12, 0x9e, 0x42, 0x3c, 0x12, 0x02, 0x00, 0x1d, 0x53, 0xde, 0x3f, 0x82, 0x48, 0x3c, 0x10, 
0x48, 0x43, 0x18, 0x92, 0x36, 0x12, 0x6f, 0x2f, 0x17, 0x42, 0x34, 0x12, 0x07, 0x58, 0x0a, 0x48, 
0x0a, 0x5a, 0x0a, 0x5a, 0x0c, 0x49, 0x7d, 0x40, 0x05, 0x00, 0xb0, 0x12, 0xa2, 0xf7, 0x0c, 0x59, 
0x0c, 0x57, 0x3c, 0x50, 0x06, 0x00, 0x0c, 0x5c, 0x0c, 0x5c, 0x9a, 0x4c, 0x40, 0x12, 0x74, 0x10, 
0x9a, 0x4c, 0x42, 0x12, 0x76, 0x10, 0x18, 0x53, 0xe4, 0x3f, 0x18, 0x42, 0x34, 0x12, 0x08, 0x5a, 
0x0c, 0x49, 0x7d, 0x40, 0x05, 0x00, 0xb0, 0x12, 0xa2, 0xf7, 0x0c, 0x59, 0x0c, 0x58, 0x3c, 0x50, 
0x06, 0x00, 0x0c, 0x5c, 0x0c, 0x5c, 0x0d, 0x4a, 0x0d, 0x5d, 0x0d, 0x5d, 0x9c, 0x4d, 0x74, 0x10, 
0x40, 0x12, 0x9c, 0x4d, 0x76, 0x10, 0x42, 0x12, 0x1a, 0x53, 0x1a, 0x92, 0x36, 0x12, 0xe5, 0x2b, 
0x7c, 0x40, 0x10, 0x00, 0xb0, 0x12, 0xc8, 0xea, 0xf2, 0x40, 0xfe, 0xff, 0x08, 0x14, 0x15, 0x3e, 
0x3c, 0x40, 0x00, 0x80, 0x30, 0x40, 0xfe, 0xf0, 0x0a, 0x12, 0x09, 0x12, 0xb2, 0x40, 0x80, 0x5a, 
0x20, 0x01, 0xf2, 0x42, 0x58, 0x00, 0xf2, 0x40, 0x05, 0x00, 0x57, 0x00, 0xe2, 0x43, 0xd8, 0x01, 
0x82, 0x43, 0xc0, 0x13, 0xd2, 0x43, 0x41, 0x12, 0xc2, 0x43, 0x40, 0x12, 0x82, 0x43, 0x48, 0x12, 
0x82, 0x43, 0x44, 0x12, 0xf2, 0xf0, 0xcf, 0xff, 0xe9, 0x01, 0x39, 0x40, 0x80, 0x13, 0x4a, 0x43, 
0xc2, 0x4a, 0xe8, 0x01, 0xe2, 0xd3, 0xe9, 0x01, 0x7c, 0x40, 0x10, 0x00, 0xb0, 0x12, 0xc2, 0xea, 
0x29, 0x53, 0x99, 0x42, 0xea, 0x01, 0xfe, 0xff, 0xe2, 0xc3, 0xe9, 0x01, 0x2a, 0x53, 0x3a, 0x90, 
0x40, 0x00, 0xee, 0x23, 0xe2, 0x92, 0x81, 0x13, 0x04, 0x20, 0x5c, 0x42, 0x8c, 0x13, 0x82, 0x4c, 
0x52, 0x10, 0x1d, 0x42, 0x52, 0x10, 0xa2, 0x43, 0xa8, 0x01, 0x3d, 0x90, 0x03, 0x00, 0x3a, 0x20, 
0xb2, 0x40, 0x35, 0x20, 0xa2, 0x01, 0xb2, 0x40, 0x30, 0x00, 0xa0, 0x01, 0xb2, 0x40, 0x03, 0x00, 
0x58, 0x12, 0xb2, 0x40, 0x03, 0x00, 0xdc, 0x12, 0x82, 0x43, 0xa8, 0x01, 0xb2, 0x40, 0x10, 0x14, 
0x68, 0x10, 0xb2, 0x40, 0x66, 0x10, 0x6c, 0x10, 0xb2, 0x40, 0x64, 0x10, 0x6e, 0x10, 0xb2, 0x40, 
0x0e, 0x14, 0x6a, 0x10, 0xb2, 0x40, 0x0a, 0x14, 0x70, 0x10, 0xb2, 0x40, 0x55, 0x12, 0x72, 0x10, 
0xb2, 0xd0, 0x03, 0x00, 0x04, 0x02, 0xc2, 0x43, 0xe2, 0x01, 0xc2, 0x43, 0xe6, 0x01, 0x3c, 0x40, 
0x00, 0x08, 0xb0, 0x12, 0xc8, 0xea, 0x82, 0x43, 0x6a, 0x13, 0x32, 0xc2, 0x03, 0x43, 0x1c, 0x42, 
0x98, 0x10, 0x0c, 0x93, 0x1a, 0x24, 0xb2, 0xb0, 0x00, 0xff, 0x98, 0x10, 0x13, 0x24, 0x32, 0xd0, 
0x18, 0x00, 0xf3, 0x3f, 0xb2, 0x40, 0x33, 0x20, 0xa2, 0x01, 0x7c, 0x42, 0x2d, 0x93, 0x02, 0x24, 
0x7c, 0x40, 0x12, 0x00, 0x3c, 0xd0, 0x40, 0x00, 0x82, 0x4c, 0xac, 0x01, 0xb2, 0x40, 0x12, 0x00, 
0xa0, 0x01, 0xbc, 0x3f, 0x32, 0xd0, 0x58, 0x00, 0xe0, 0x3f, 0x32, 0xd0, 0xf8, 0x00, 0xdd, 0x3f, 
0x0d, 0x12, 0x0c, 0x12, 0x92, 0x43, 0xa8, 0x01, 0x1c, 0x42, 0x8a, 0x10, 0x0d, 0x4c, 0x1d, 0x53, 
0x82, 0x4d, 0x8a, 0x10, 0x0c, 0x93, 0x1a, 0x20, 0x5c, 0x42, 0xbe, 0x13, 0x6c, 0x93, 0x16, 0x20, 
0x5c, 0x42, 0xbf, 0x13, 0x6c, 0x93, 0x12, 0x20, 0xd2, 0x43, 0xbe, 0x13, 0xc2, 0x43, 0xbf, 0x13, 
0x5d, 0x42, 0xbe, 0x13, 0x5c, 0x42, 0xbf, 0x13, 0xc2, 0x4d, 0xc0, 0x01, 0xc2, 0x4c, 0xc1, 0x01, 
0xb1, 0xc0, 0xf0, 0x00, 0x04, 0x00, 0x3c, 0x41, 0x3d, 0x41, 0x00, 0x13, 0x5c, 0x42, 0xbe, 0x13, 
0x0c, 0x93, 0xee, 0x27, 0xd2, 0x53, 0xbf, 0x13, 0x5c, 0x42, 0xbf, 0x13, 0x7d, 0x40, 0x07, 0x00, 
0x4d, 0x9c, 0xe6, 0x2f, 0xc2, 0x43, 0xbf, 0x13, 0xd2, 0x53, 0xbe, 0x13, 0x5c, 0x42, 0xbe, 0x13, 
0x4d, 0x9c, 0xde, 0x2f, 0xd2, 0x43, 0xbe, 0x13, 0xdb, 0x3f, 0x0f, 0x4c, 0x7c, 0x40, 0x11, 0x00, 
0x5b, 0x43, 0x0d, 0x9f, 0x05, 0x2c, 0x3c, 0x53, 0x0c, 0x93, 0x05, 0x24, 0x0d, 0x93, 0x07, 0x34, 
0x4c, 0x43, 0x0b, 0x93, 0x07, 0x20, 0x0e, 0x93, 0x01, 0x24, 0x0c, 0x4f, 0x30, 0x41, 0x0d, 0x5d, 
0x0b, 0x5b, 0xef, 0x3f, 0x0f, 0x9d, 0x02, 0x28, 0x0f, 0x8d, 0x0c, 0xdb, 0x12, 0xc3, 0x0b, 0x10, 
0x12, 0xc3, 0x0d, 0x10, 0xee, 0x3f, 0x4e, 0x43, 0xb0, 0x12, 0x4a, 0xf7, 0x30, 0x41, 0x34, 0x41, 
0x35, 0x41, 0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39, 0x41, 0x3a, 0x41, 0x30, 0x41, 0x3d, 0x53, 
0x0c, 0x5c, 0x0d, 0x93, 0xfc, 0x23, 0x30, 0x41, 0x3d, 0x53, 0x0c, 0x11, 0x0d, 0x93, 0xfc, 0x23, 
0x30, 0x41, 0x3d, 0x53, 0x12, 0xc3, 0x0c, 0x10, 0x0d, 0x93, 0xfb, 0x23, 0x30, 0x41, 0x02, 0x12, 
0x32, 0xc2, 0x03, 0x43, 0x82, 0x4c, 0x30, 0x01, 0x82, 0x4d, 0x38, 0x01, 0x1c, 0x42, 0x3a, 0x01, 
0x00, 0x13, 0x0a, 0x12, 0x09, 0x12, 0x0f, 0x4d, 0x0f, 0x5e, 0x0d, 0x9c, 0x02, 0x2c, 0x0c, 0x9f, 
0x07, 0x28, 0x0e, 0x4c, 0x0d, 0x9f, 0x0a, 0x24, 0xfe, 0x4d, 0x00, 0x00, 0x1e, 0x53, 0xfa, 0x3f, 
0x09, 0x4e, 0x39, 0xe3, 0x4d, 0x43, 0x3d, 0x53, 0x09, 0x9d, 0x01, 0x20, 0xcd, 0x3f, 0x0b, 0x4e, 
0x0b, 0x5d, 0x0b, 0x5c, 0x0a, 0x4f, 0x0a, 0x5d, 0xeb, 0x4a, 0x00, 0x00, 0xf4, 0x3f, 0x0e, 0x5c, 
0x0f, 0x4c, 0x0f, 0x9e, 0x01, 0x20, 0x30, 0x41, 0x1f, 0x53, 0xcf, 0x4d, 0xff, 0xff, 0xf9, 0x3f, 
};

const unsigned char icu_init_fw_vec[ICU_INIT_VEC_SIZE] = {
0xdc, 0xeb, 0xde, 0xeb, 0xd0, 0xf6, 0x5c, 0xec, 0x30, 0xec, 0x04, 0xec, 0xd8, 0xeb, 0xda, 0xeb, 
0x42, 0xf0, 0x34, 0xf0, 0xe0, 0xeb, 0x84, 0xec, 0xf0, 0xef, 0x02, 0xed, 0xd6, 0xeb, 0x00, 0xe8, 
};

