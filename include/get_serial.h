/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Serial Number Header
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Get device serial number as a hex string
 *
 * @param serial  Buffer to store the serial number string
 * @param max_len Maximum length of the buffer (should be at least 17 for 8-byte ID)
 */
void get_serial_string(char *serial, size_t max_len);

/**
 * @brief Get device serial number as raw bytes
 *
 * @param buffer  Buffer to store the raw serial bytes
 * @param max_len Maximum length of the buffer
 * @return        Number of bytes written
 */
size_t get_serial_raw(uint8_t *buffer, size_t max_len);
