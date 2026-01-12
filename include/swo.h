/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - SWO (Serial Wire Output) Header
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* SWO Transport modes */
#define SWO_TRANSPORT_NONE      0
#define SWO_TRANSPORT_DATA      1
#define SWO_TRANSPORT_WINUSB    2

/* SWO Modes */
#define SWO_MODE_OFF            0
#define SWO_MODE_UART           1
#define SWO_MODE_MANCHESTER     2

/* SWO Status bits */
#define SWO_STATUS_CAPTURE      (1U << 0)
#define SWO_STATUS_ERROR        (1U << 6)
#define SWO_STATUS_OVERRUN      (1U << 7)

/**
 * @brief Initialize SWO capture
 *
 * @return 0 on success, negative error code otherwise
 */
int swo_init(void);

/**
 * @brief Set SWO transport mode
 *
 * @param transport Transport mode (NONE, DATA, WINUSB)
 * @return 0 on success, negative error code otherwise
 */
int swo_set_transport(uint8_t transport);

/**
 * @brief Get current SWO transport mode
 *
 * @return Current transport mode
 */
uint8_t swo_get_transport(void);

/**
 * @brief Set SWO mode
 *
 * @param mode SWO mode (OFF, UART, MANCHESTER)
 * @return 0 on success, negative error code otherwise
 */
int swo_set_mode(uint8_t mode);

/**
 * @brief Get current SWO mode
 *
 * @return Current SWO mode
 */
uint8_t swo_get_mode(void);

/**
 * @brief Set SWO baudrate
 *
 * @param baudrate Desired baudrate in Hz
 * @return Actual baudrate achieved
 */
uint32_t swo_set_baudrate(uint32_t baudrate);

/**
 * @brief Get current SWO baudrate
 *
 * @return Current baudrate in Hz
 */
uint32_t swo_get_baudrate(void);

/**
 * @brief Start SWO capture
 *
 * @return 0 on success, negative error code otherwise
 */
int swo_start(void);

/**
 * @brief Stop SWO capture
 *
 * @return 0 on success, negative error code otherwise
 */
int swo_stop(void);

/**
 * @brief Get SWO status
 *
 * @return Status byte (CAPTURE, ERROR, OVERRUN flags)
 */
uint8_t swo_get_status(void);

/**
 * @brief Get SWO trace count
 *
 * @return Number of bytes in trace buffer
 */
uint32_t swo_get_count(void);

/**
 * @brief Read SWO trace data
 *
 * @param buffer Buffer to store trace data
 * @param max_len Maximum bytes to read
 * @return Number of bytes actually read
 */
uint32_t swo_read_data(uint8_t *buffer, uint32_t max_len);

/**
 * @brief Get SWO buffer size
 *
 * @return Size of SWO capture buffer in bytes
 */
uint32_t swo_get_buffer_size(void);
