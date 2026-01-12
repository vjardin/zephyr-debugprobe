/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
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

/* UART Bridge Pins (directly active UART1 on J2 connector) */
#define PROBE_PIN_UART_TX       4       /* Target UART TX */
#define PROBE_PIN_UART_RX       5       /* Target UART RX */

/*
 * LED Pins - matching original debugprobe firmware
 *   D1 (Red, GPIO2):     USB connected indicator
 *   D2 (Green, GPIO7):   UART RX activity
 *   D3 (Yellow, GPIO8):  UART TX activity
 *   D4 (Green, GPIO15):  DAP connected
 *   D5 (Yellow, GPIO16): DAP running/activity
 */
#define PROBE_PIN_LED_USB       2       /* D1 Red - USB connected */
#define PROBE_PIN_LED_UART_RX   7       /* D2 Green - UART RX activity */
#define PROBE_PIN_LED_UART_TX   8       /* D3 Yellow - UART TX activity */
#define PROBE_PIN_LED_DAP_CONN  15      /* D4 Green - DAP connected */
#define PROBE_PIN_LED_DAP_RUN   16      /* D5 Yellow - DAP running */

/* Legacy aliases */
#define PROBE_PIN_LED_DAP       PROBE_PIN_LED_DAP_RUN
#define PROBE_PIN_LED_CONNECTED PROBE_PIN_LED_DAP_CONN
#define PROBE_PIN_LED_RUNNING   PROBE_PIN_LED_DAP_RUN

/* Level Shifter Control */
#define PROBE_PIN_IO_OEN        10      /* Output enable for level shifters */

/* UART Interface */
#define PROBE_UART_INTERFACE    uart1

/* Default Clock Speed */
#define PROBE_DEFAULT_CLOCK_HZ  1000000 /* 1 MHz default SWD clock */

/* Maximum Clock Speed */
#define PROBE_MAX_CLOCK_HZ      24000000 /* 24 MHz max with PIO */
