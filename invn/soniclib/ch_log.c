/*! \file ch_log.c
 \brief Internal driver functions to log informations on console

 The user should not need to edit this file.
 */

/*
 Copyright 2016-2023, InvenSense, Inc.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED.

 */
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <invn/soniclib/ch_log.h>
#include <invn/soniclib/chirp_bsp.h>

#define MSG_BUFFER_LENGTH   200
#define MODULE_NAME_LENGTH  7 /* log module name max size in char */
#define FUNC_NAME_LENGTH    30
#define HEADER_LENGTH       (1 + 1 + MODULE_NAME_LENGTH + 1 + FUNC_NAME_LENGTH + 1)
#define INDEX_LOG_MESSAGE   (HEADER_LENGTH - 1)
#define END_OF_LINE_PATTERN "\r\n"
#define END_OF_LINE_LENGTH  2

/* Get size available in buffer removing header length and the \r\n at the end of the string. */
#define SIZEOF_BUF_FOR_DATA(buf) (sizeof(buf) - (HEADER_LENGTH + END_OF_LINE_LENGTH))

static char buf_msg[MSG_BUFFER_LENGTH];

static inline void add_header(const char level, const char *name, const char *func_name, char *out_buf) {
	/* Write the header :
	   - 1 letter for the criticality,
	   - 1 space,
	   - module name space complemented if less than MODULE_NAME_LENGTH,
	   - 1 space,
	   - '\0' char */
	snprintf(out_buf, HEADER_LENGTH + 1, "%c %-*.*s %-*.*s ", level, MODULE_NAME_LENGTH, MODULE_NAME_LENGTH, name,
	         FUNC_NAME_LENGTH, FUNC_NAME_LENGTH, func_name);
}

static inline void add_eof(char *out_buf, size_t out_buf_len) {
	uint32_t len;

	/* Add the carriage return */
	out_buf[out_buf_len - 1] = '\0'; /* be sure there is at least one '\0' in buffer */
	len                      = strlen(out_buf);
	snprintf(&out_buf[len], out_buf_len, END_OF_LINE_PATTERN);
}

__attribute__((format(printf, 1, 2))) void ch_log_printf(const char *format, ...) {
	char buf[MSG_BUFFER_LENGTH] = {0};
	va_list va;

	va_start(va, format);
	vsnprintf(&buf[0], sizeof(buf), format, va);
	va_end(va);

	chbsp_print_str(buf);
}

__attribute__((format(printf, 1, 2))) void ch_log_printf_eol(const char *format, ...) {
	char buf[MSG_BUFFER_LENGTH] = {0};
	va_list va;

	va_start(va, format);
	vsnprintf(&buf[0], sizeof(buf), format, va);
	va_end(va);

	add_eof(buf, sizeof(buf));

	chbsp_print_str(buf);
}

__attribute__((format(printf, 4, 5))) void ch_log_prefix_printf(const char level, const char *name,
                                                                const char *func_name, const char *format, ...) {
	char buf[MSG_BUFFER_LENGTH] = {0};
	va_list va;

	add_header(level, name, func_name, &buf[0]);

	va_start(va, format);
	/* Copy all after the header */
	vsnprintf(&buf[INDEX_LOG_MESSAGE], SIZEOF_BUF_FOR_DATA(buf), format, va);
	va_end(va);

	add_eof(buf, sizeof(buf));

	chbsp_print_str(buf);
}

__attribute__((format(printf, 4, 5))) void ch_log_prefix_start(const char level, const char *name,
                                                               const char *func_name, const char *format, ...) {
	va_list va;

	add_header(level, name, func_name, &buf_msg[0]);

	va_start(va, format);
	/* Copy all after the header. Save room for the \r\n at the end of the string. */
	vsnprintf(&buf_msg[INDEX_LOG_MESSAGE], SIZEOF_BUF_FOR_DATA(buf_msg), format, va);
	va_end(va);
	buf_msg[MSG_BUFFER_LENGTH - 1] = '\0';
}

__attribute__((format(printf, 1, 2))) void ch_log_start(const char *format, ...) {
	va_list va;

	va_start(va, format);
	/* Copy all after the header. Save room for the \r\n at the end of the string. */
	vsnprintf(&buf_msg[0], SIZEOF_BUF_FOR_DATA(buf_msg), format, va);
	va_end(va);
	buf_msg[MSG_BUFFER_LENGTH - 1] = '\0';
}

__attribute__((format(printf, 1, 2))) void ch_log_msg(const char *format, ...) {
	va_list va;
	uint32_t len;

	buf_msg[sizeof(buf_msg) - 1] = '\0'; /* be sure there is at least one '\0' in buffer */
	len                          = strlen(buf_msg);
	va_start(va, format);
	/* Copy all after the header. Save room for the \r\n at the end of the string. */
	vsnprintf(&buf_msg[len], SIZEOF_BUF_FOR_DATA(buf_msg), format, va);
	va_end(va);
}

void ch_log_end(void) {
	add_eof(buf_msg, sizeof(buf_msg));

	chbsp_print_str(buf_msg);
}