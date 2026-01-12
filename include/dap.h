/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - CMSIS-DAP Header
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>

/* DP Register addresses */
#define DAP_DP_DPIDR        0x00
#define DAP_DP_CTRL_STAT    0x04
#define DAP_DP_SELECT       0x08
#define DAP_DP_RDBUFF       0x0C

/* CTRL/STAT register bits */
#define DAP_SWD_ORUNDETECT  (1U << 0)
#define DAP_SWD_STICKYORUN  (1U << 1)
#define DAP_SWD_TRNMODE     (3U << 2)
#define DAP_SWD_STICKYCMP   (1U << 4)
#define DAP_SWD_STICKYERR   (1U << 5)
#define DAP_SWD_READOK      (1U << 6)
#define DAP_SWD_WDATAERR    (1U << 7)
#define DAP_SWD_MASKLANE    (0xFU << 8)
#define DAP_SWD_TRNCNT      (0xFFFU << 12)
#define DAP_SWD_CDBGRSTREQ  (1U << 26)
#define DAP_SWD_CDBGRSTACK  (1U << 27)
#define DAP_SWD_CDBGPWRUPREQ (1U << 28)
#define DAP_SWD_CDBGPWRUPACK (1U << 29)
#define DAP_SWD_CSYSPWRUPREQ (1U << 30)
#define DAP_SWD_CSYSPWRUPACK (1U << 31)

/**
 * @brief Initialize DAP
 */
void dap_init(void);

/**
 * @brief Process DAP request
 *
 * @param request      Pointer to request buffer
 * @param request_len  Length of request
 * @param response     Pointer to response buffer
 * @param response_max Maximum response length
 * @return             Actual response length
 */
uint32_t dap_process_request(const uint8_t *request, uint32_t request_len,
                             uint8_t *response, uint32_t response_max);

/**
 * @brief Process vendor-specific DAP command
 *
 * @param request  Pointer to request buffer
 * @param response Pointer to response buffer
 * @return         Response length
 */
uint32_t dap_vendor_command(const uint8_t *request, uint8_t *response);

/**
 * @brief Get configured idle cycles count
 *
 * Returns the number of idle cycles to insert after each transfer,
 * as configured by DAP_TransferConfigure command.
 *
 * @return Number of idle cycles
 */
uint8_t dap_get_idle_cycles(void);

/**
 * @brief Get configured SWD turnaround period
 *
 * Returns the turnaround period (1-4 clock cycles) as configured
 * by DAP_SWD_Configure command.
 *
 * @return Turnaround period in clock cycles
 */
uint8_t dap_get_swd_turnaround(void);

/**
 * @brief Get configured SWD data phase flag
 *
 * Returns whether data phase should be generated on WAIT/FAULT,
 * as configured by DAP_SWD_Configure command.
 *
 * @return 1 if data phase should be generated, 0 otherwise
 */
uint8_t dap_get_swd_data_phase(void);

/**
 * @brief Get host-controlled running LED state
 *
 * Returns the running LED state as set by the host via DAP_HostStatus.
 * Used by USB layer to decide whether to show activity indication.
 *
 * @return true if host set running LED on, false otherwise
 */
bool dap_get_running_led_state(void);
