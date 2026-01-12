/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Example Board Configuration
 *
 * Template for creating custom board configurations.
 * Copy this file and modify pin assignments for your hardware.
 */

#pragma once

/*
 * Custom Board Pin Assignments
 *
 * Modify these definitions to match your hardware setup.
 * All pins use GPIO numbers.
 */

/* SWD Interface Pins (REQUIRED) */
#define PROBE_PIN_SWCLK         2       /* SWD clock output */
#define PROBE_PIN_SWDIO         3       /* SWD data I/O */

/* Optional SWD pins - set to (-1) if not available */
#define PROBE_PIN_SWDIR         (-1)    /* Direction control for level shifter */
#define PROBE_PIN_RESET         (-1)    /* Target nRESET */

/* JTAG Interface Pins (OPTIONAL - TCK/TMS shared with SWD) */
#define PROBE_PIN_TCK           PROBE_PIN_SWCLK  /* TCK = SWCLK */
#define PROBE_PIN_TMS           PROBE_PIN_SWDIO  /* TMS = SWDIO */
#define PROBE_PIN_TDI           (-1)    /* TDI output to target */
#define PROBE_PIN_TDO           (-1)    /* TDO input from target */
#define PROBE_PIN_TRST          (-1)    /* nTRST (active low reset) */

/* UART Bridge Pins (OPTIONAL - set to -1 to disable) */
#define PROBE_PIN_UART_TX       4       /* UART TX to target */
#define PROBE_PIN_UART_RX       5       /* UART RX from target */

/* LED Pins (OPTIONAL - set to -1 if not available) */
#define PROBE_PIN_LED_DAP       (-1)    /* DAP activity LED */
#define PROBE_PIN_LED_UART_TX   (-1)    /* UART TX activity */
#define PROBE_PIN_LED_UART_RX   (-1)    /* UART RX activity */
#define PROBE_PIN_LED_CONNECTED (-1)    /* Target connected indicator */
#define PROBE_PIN_LED_RUNNING   (-1)    /* Target running indicator */

/* Level Shifter Control (OPTIONAL - set to -1 if not used) */
#define PROBE_PIN_IO_OEN        (-1)    /* Output enable for level shifters */

/* UART Interface Selection */
#define PROBE_UART_INTERFACE    uart0   /* uart0 or uart1 */

/* Clock Configuration */
#define PROBE_DEFAULT_CLOCK_HZ  1000000 /* 1 MHz default SWD clock */
#define PROBE_MAX_CLOCK_HZ      10000000 /* Maximum SWD clock speed */

/*
 * Notes:
 *
 * 1. SWCLK and SWDIO are the minimum required pins for SWD debugging.
 *
 * 2. If using a level shifter, configure PROBE_PIN_SWDIR to control
 *    the direction (high = output, low = input for SWDIO).
 *
 * 3. Target reset (nRESET) is optional but recommended for reliable
 *    target connection.
 *
 * 4. UART pins enable the CDC-UART bridge feature for serial console
 *    access to the target.
 *
 * 5. LEDs provide visual feedback but are not required for operation.
 *
 * 6. The level shifter output enable (IO_OEN) can be used to
 *    completely disable the debug interface when not in use.
 *
 * 7. JTAG requires TDI and TDO pins in addition to TCK/TMS (which are
 *    shared with SWCLK/SWDIO). If TDI/TDO are not available (-1),
 *    JTAG mode will have limited functionality (no data transfer).
 *
 * 8. nTRST is an optional JTAG reset signal (active low). Most targets
 *    don't require it as the TAP can be reset via TMS sequence.
 */
