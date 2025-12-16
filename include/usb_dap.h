/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - USB DAP Interface Header
 *
 * CMSIS-DAP v2 USB bulk endpoint handler.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize USB DAP interface
 *
 * Sets up the USB bulk endpoints for CMSIS-DAP v2 communication
 * and starts the processing thread.
 *
 * @return 0 on success, negative error code otherwise
 */
int usb_dap_init(void);

/**
 * @brief Write response data to USB
 *
 * @param data Pointer to data buffer
 * @param len  Length of data to write
 * @return 0 on success, negative error code otherwise
 */
int usb_dap_write(const uint8_t *data, uint32_t len);

/**
 * @brief Check if USB DAP is ready
 *
 * @return true if USB is configured and ready for communication
 */
bool usb_dap_ready(void);
