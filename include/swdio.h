/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - SWD I/O Header
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize PIO for SWD (if available)
 *
 * @return 0 on success, negative error code otherwise
 */
int swdio_pio_init(void);

/**
 * @brief Set PIO clock frequency
 *
 * @param freq_hz Desired SWD clock frequency in Hz
 */
void swdio_pio_set_clock(uint32_t freq_hz);

/**
 * @brief Write data using PIO
 *
 * @param data Data to write
 * @param bits Number of bits to write
 */
void swdio_pio_write(uint32_t data, uint32_t bits);

/**
 * @brief Read data using PIO
 *
 * @param bits Number of bits to read
 * @return Read data
 */
uint32_t swdio_pio_read(uint32_t bits);

/**
 * @brief Check if PIO is available
 *
 * @return true if PIO is initialized and available
 */
bool swdio_pio_available(void);
