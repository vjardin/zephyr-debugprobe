/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Device Watchdog Header
 *
 * Monitors USB state and system health, handles reconnection.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize device watchdog
 *
 * Starts the watchdog monitoring thread which checks:
 * - USB connection state
 * - System health indicators
 * - Automatic recovery from error states
 *
 * @return 0 on success, negative error code otherwise
 */
int watchdog_init(void);

/**
 * @brief Feed the watchdog
 *
 * Call periodically from main processing loops to indicate
 * the system is functioning normally.
 */
void watchdog_feed(void);

/**
 * @brief Check if USB is connected
 *
 * @return true if USB is connected and enumerated
 */
bool watchdog_usb_connected(void);

/**
 * @brief Get uptime in milliseconds
 *
 * @return System uptime since boot
 */
uint32_t watchdog_get_uptime_ms(void);

/**
 * @brief Get error count
 *
 * @return Number of errors detected since boot
 */
uint32_t watchdog_get_error_count(void);

/**
 * @brief Trigger system reset
 *
 * Performs a controlled system reset after cleanup.
 */
void watchdog_reset(void);
