/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - CDC UART Bridge Header
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the CDC UART bridge
 *
 * Sets up UART hardware, interrupts, and buffers
 */
void cdc_uart_init(void);

/**
 * @brief CDC UART task function
 *
 * Should be called periodically to process data transfers
 * between USB CDC and hardware UART
 */
void cdc_uart_task(void);

/**
 * @brief Write data to UART (from CDC)
 *
 * @param data Pointer to data buffer
 * @param len  Number of bytes to write
 */
void cdc_uart_write(const uint8_t *data, size_t len);

/**
 * @brief Read data from UART (for CDC)
 *
 * @param data    Pointer to buffer for received data
 * @param max_len Maximum number of bytes to read
 * @return Number of bytes actually read
 */
size_t cdc_uart_read(uint8_t *data, size_t max_len);

/**
 * @brief Check if data is available from UART
 *
 * @return true if data is available, false otherwise
 */
bool cdc_uart_data_available(void);
