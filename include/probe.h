/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - SWD Probe Header
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* DAP Transfer definitions - see DAP_config.h for full definitions */
#include "DAP_config.h"

/**
 * @brief Initialize probe GPIO pins
 *
 * @return 0 on success, negative error code otherwise
 */
int probe_gpio_init(void);

/**
 * @brief Set SWJ clock frequency
 *
 * @param clock_hz Clock frequency in Hz
 */
void probe_set_swj_clock(uint32_t clock_hz);

/**
 * @brief Get current SWJ clock frequency
 *
 * @return Clock frequency in Hz
 */
uint32_t probe_get_swj_clock(void);

/**
 * @brief Execute SWJ sequence
 *
 * Output a sequence of bits on SWDIO with SWCLK
 *
 * @param count Number of bits to output
 * @param data  Bit sequence data (LSB first)
 */
void probe_swj_sequence(uint32_t count, const uint8_t *data);

/**
 * @brief Perform SWD transfer
 *
 * @param request Request byte (APnDP, RnW, A[2:3], parity)
 * @param data    Pointer to data for write, or result for read
 * @return        ACK value (OK=1, WAIT=2, FAULT=4) or error
 */
uint8_t probe_swd_transfer(uint32_t request, uint32_t *data);

/**
 * @brief Control target reset
 *
 * @param assert true to assert reset, false to release
 */
void probe_set_reset(bool assert);

/**
 * @brief Enable or disable SWD port
 *
 * @param enable true to enable, false to disable
 */
void probe_port_enable(bool enable);

/**
 * @brief Check if target is connected
 *
 * @return true if connected, false otherwise
 */
bool probe_is_connected(void);

/**
 * @brief Put SWD pins in high-impedance state
 */
void probe_pins_hiz(void);

/**
 * @brief Restore SWD pins to active state
 */
void probe_pins_active(void);

/**
 * @brief Read a single bit from SWDIO
 *
 * @return Bit value (0 or 1)
 */
int probe_read_bit(void);

/**
 * @brief Write a single bit to SWDIO
 *
 * @param bit Bit value to write
 */
void probe_write_bit(int bit);

/*
 * JTAG Support
 */

/**
 * @brief Initialize JTAG mode
 *
 * Configures GPIO pins for JTAG operation (TCK, TMS, TDI, TDO).
 *
 * @return 0 on success, negative error code otherwise
 */
int probe_jtag_init(void);

/**
 * @brief Deinitialize JTAG mode
 *
 * Releases JTAG pins and returns them to high-Z state.
 */
void probe_jtag_deinit(void);

/**
 * @brief Check if full JTAG is supported
 *
 * @return true if TDI and TDO pins are available
 */
bool probe_jtag_available(void);

/**
 * @brief Configure JTAG chain
 *
 * @param count Number of devices in chain
 * @param ir_length Array of IR lengths for each device
 */
void probe_jtag_configure(uint8_t count, const uint8_t *ir_length);

/**
 * @brief Execute JTAG sequence
 *
 * @param info Sequence info byte:
 *             - Bits 0-5: TCK count (0 = 64 clocks)
 *             - Bit 6: TMS value during sequence
 *             - Bit 7: TDO capture enable
 * @param tdi  TDI data to output (LSB first)
 * @param tdo  Buffer for captured TDO data (if bit 7 set)
 */
void probe_jtag_sequence(uint8_t info, const uint8_t *tdi, uint8_t *tdo);

/**
 * @brief Read JTAG IDCODE
 *
 * Reads the IDCODE from a device in the JTAG chain.
 *
 * @param index Device index in chain
 * @param idcode Pointer to store IDCODE
 * @return 0 on success, negative error code otherwise
 */
int probe_jtag_read_idcode(uint8_t index, uint32_t *idcode);

/**
 * @brief Write JTAG IR (Instruction Register)
 *
 * @param index Device index in chain
 * @param ir Instruction value
 * @param ir_len Instruction length in bits
 * @return 0 on success, negative error code otherwise
 */
int probe_jtag_write_ir(uint8_t index, uint32_t ir, uint8_t ir_len);

/**
 * @brief Transfer JTAG DR (Data Register)
 *
 * @param index Device index in chain
 * @param dr_out Data to shift out (can be NULL for read-only)
 * @param dr_in Buffer for data shifted in (can be NULL for write-only)
 * @param dr_len Data register length in bits
 * @return 0 on success, negative error code otherwise
 */
int probe_jtag_transfer_dr(uint8_t index, const uint32_t *dr_out, uint32_t *dr_in, uint8_t dr_len);

/*
 * Diagnostic Functions
 */

/**
 * @brief Get SWD pin states
 *
 * @param swclk Pointer to store SWCLK state (0/1), or NULL
 * @param swdio Pointer to store SWDIO state (0/1), or NULL
 * @param reset Pointer to store nRESET state (0/1/-1 if N/A), or NULL
 */
void probe_get_pin_state(int *swclk, int *swdio, int *reset);

/**
 * @brief Get SWD pin numbers
 *
 * @param swclk_pin Pointer to store SWCLK GPIO number, or NULL
 * @param swdio_pin Pointer to store SWDIO GPIO number, or NULL
 * @param reset_pin Pointer to store nRESET GPIO number (-1 if N/A), or NULL
 */
void probe_get_pin_info(int *swclk_pin, int *swdio_pin, int *reset_pin);

/**
 * @brief Check if PIO acceleration is enabled
 *
 * @return true if PIO is in use, false for GPIO bit-bang
 */
bool probe_is_pio_enabled(void);

/**
 * @brief Enable/disable PIO acceleration (for debugging)
 *
 * @param enable true to use PIO, false for GPIO bit-bang
 */
void probe_set_pio_enabled(bool enable);

/**
 * @brief Get system clock frequency
 *
 * @return System clock in Hz
 */
uint32_t probe_get_sys_clock(void);
