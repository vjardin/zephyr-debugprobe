/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - Raspberry Pi Pico Board Configuration
 *
 * Pin configuration for using a Raspberry Pi Pico as a debug probe.
 * This is the "Debug on Pico" mode from the original debugprobe.
 */

#pragma once

/*
 * Pico Board Pin Assignments (Debug on Pico mode)
 * These pins are suitable for a standard Pico board used as debugger.
 */

/* SWD Interface Pins */
#define PROBE_PIN_SWCLK         2
#define PROBE_PIN_SWDIO         3
#define PROBE_PIN_SWDIR         (-1)    /* Direction control (optional) */
#define PROBE_PIN_RESET         6       /* Target nRESET */

/* JTAG Interface Pins (TCK/TMS shared with SWD) */
#define PROBE_PIN_TCK           PROBE_PIN_SWCLK  /* TCK = SWCLK (GP2) */
#define PROBE_PIN_TMS           PROBE_PIN_SWDIO  /* TMS = SWDIO (GP3) */
#define PROBE_PIN_TDI           4       /* TDI output to target */
#define PROBE_PIN_TDO           5       /* TDO input from target */
#define PROBE_PIN_TRST          (-1)    /* nTRST not connected */

/* UART Bridge Pins (uart0) */
#define PROBE_PIN_UART_TX       0       /* GP0 = UART0 TX */
#define PROBE_PIN_UART_RX       1       /* GP1 = UART0 RX */

/* LED Pins - Pico has only one LED on GP25 */
#define PROBE_PIN_LED_DAP       25      /* On-board LED for activity */
#define PROBE_PIN_LED_UART_TX   (-1)    /* Not available */
#define PROBE_PIN_LED_UART_RX   (-1)    /* Not available */
#define PROBE_PIN_LED_CONNECTED (-1)    /* Not available */
#define PROBE_PIN_LED_RUNNING   (-1)    /* Not available */

/* No level shifter on standard Pico */
#define PROBE_PIN_IO_OEN        (-1)

/* UART Interface */
#define PROBE_UART_INTERFACE    uart0

/* Default Clock Speed */
#define PROBE_DEFAULT_CLOCK_HZ  1000000 /* 1 MHz default SWD clock */

/* Maximum Clock Speed - limited without level shifter */
#define PROBE_MAX_CLOCK_HZ      10000000 /* 10 MHz max on Pico */

/*
 * Alternative pin configurations
 *
 * If you need different pins, define these before including this file:
 *
 * #define PICO_DEBUG_SWCLK_PIN  2
 * #define PICO_DEBUG_SWDIO_PIN  3
 * #define PICO_DEBUG_RESET_PIN  5
 */

#ifdef PICO_DEBUG_SWCLK_PIN
#undef PROBE_PIN_SWCLK
#define PROBE_PIN_SWCLK PICO_DEBUG_SWCLK_PIN
#endif

#ifdef PICO_DEBUG_SWDIO_PIN
#undef PROBE_PIN_SWDIO
#define PROBE_PIN_SWDIO PICO_DEBUG_SWDIO_PIN
#endif

#ifdef PICO_DEBUG_RESET_PIN
#undef PROBE_PIN_RESET
#define PROBE_PIN_RESET PICO_DEBUG_RESET_PIN
#endif
