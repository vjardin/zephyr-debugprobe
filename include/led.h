/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - LED Control
 *
 * LED behavior matching original Raspberry Pi debugprobe:
 *   - D1 (Red, GPIO2):     USB connected indicator
 *   - D2 (Green, GPIO7):   UART RX activity
 *   - D3 (Yellow, GPIO8):  UART TX activity
 *   - D4 (Green, GPIO15):  DAP connected
 *   - D5 (Yellow, GPIO16): DAP running/activity
 */

#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize all LEDs.
 *
 * @return 0 on success, negative error code on failure
 */
int led_init(void);

/**
 * Set USB connected LED state (D1 Red).
 * Called when USB configuration changes.
 *
 * @param connected true when USB is configured and ready
 */
void led_usb_connected(bool connected);

/**
 * Set DAP connected LED state (D4 Green).
 * Called when target connection state changes.
 *
 * @param connected true when DAP is connected to target
 */
void led_dap_connected(bool connected);

/**
 * Set DAP running LED state (D5 Yellow).
 * Called during DAP activity.
 *
 * @param running true when DAP is actively processing commands
 */
void led_dap_running(bool running);

/**
 * Signal UART TX activity (D3 Yellow).
 * LED stays on briefly after activity stops (debounce).
 */
void led_uart_tx_activity(void);

/**
 * Signal UART RX activity (D2 Green).
 * LED stays on briefly after activity stops (debounce).
 */
void led_uart_rx_activity(void);

/**
 * Update LED debounce timers.
 * Should be called periodically (e.g., every 1ms) from a task.
 */
void led_task(void);

/**
 * Set debounce duration for activity LEDs.
 * Default is 40ms. Can be adjusted based on baud rate.
 *
 * @param ticks_ms debounce duration in milliseconds
 */
void led_set_debounce(uint32_t ticks_ms);

/**
 * Signal SWD error condition (blinks D1 Red LED).
 * Used to indicate wiring problems or repeated transfer failures.
 * Blinks for a short duration then stops automatically.
 */
void led_swd_error(void);

/**
 * Clear SWD error indication.
 * Called when SWD transfers start succeeding again.
 */
void led_swd_error_clear(void);

#endif /* LED_H */
