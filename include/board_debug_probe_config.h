/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - Debug Probe Hardware Configuration
 *
 * Pin configuration for the Raspberry Pi Debug Probe hardware.
 */

#pragma once

/*
 * Debug Probe Hardware Pin Assignments
 * Reference: https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html
 */

/* SWD Interface Pins */
#define PROBE_PIN_SWCLK         11
#define PROBE_PIN_SWDIO         12
#define PROBE_PIN_SWDIR         13      /* Direction control for level shifter */
#define PROBE_PIN_RESET         14      /* Target nRESET */

/* JTAG Interface Pins (TCK/TMS shared with SWD) */
/* Note: Debug Probe hardware doesn't expose TDI/TDO, set to -1 */
#define PROBE_PIN_TCK           PROBE_PIN_SWCLK  /* TCK = SWCLK */
#define PROBE_PIN_TMS           PROBE_PIN_SWDIO  /* TMS = SWDIO */
#define PROBE_PIN_TDI           (-1)    /* TDI not available on Debug Probe */
#define PROBE_PIN_TDO           (-1)    /* TDO not available on Debug Probe */
#define PROBE_PIN_TRST          (-1)    /* nTRST not available */

/* UART Bridge Pins */
#define PROBE_PIN_UART_TX       8
#define PROBE_PIN_UART_RX       9

/* LED Pins */
#define PROBE_PIN_LED_DAP       15      /* DAP activity LED */
#define PROBE_PIN_LED_UART_TX   16      /* UART TX activity */
#define PROBE_PIN_LED_UART_RX   17      /* UART RX activity */
#define PROBE_PIN_LED_CONNECTED 18      /* Target connected */
#define PROBE_PIN_LED_RUNNING   19      /* Target running */

/* Level Shifter Control */
#define PROBE_PIN_IO_OEN        20      /* Output enable for level shifters */

/* UART Interface */
#define PROBE_UART_INTERFACE    uart1

/* Default Clock Speed */
#define PROBE_DEFAULT_CLOCK_HZ  1000000 /* 1 MHz default SWD clock */

/* Maximum Clock Speed */
#define PROBE_MAX_CLOCK_HZ      24000000 /* 24 MHz max with PIO */
