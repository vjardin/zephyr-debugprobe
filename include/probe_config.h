/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - Configuration Header
 *
 * Port of Raspberry Pi Debug Probe to Zephyr RTOS
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/*
 * Pin Configuration
 * These can be overridden via devicetree or Kconfig
 */

/* Debug Probe Hardware Pin Assignments */
#if defined(CONFIG_BOARD_RPI_PICO) || defined(CONFIG_BOARD_RPI_PICO_W)
/* Pico board configuration (DEBUG_ON_PICO mode) */
#define PROBE_PIN_OFFSET        2     /* Base pin for PIO consecutive pins */
#define PROBE_SWCLK_PIN         2
#define PROBE_SWDIO_PIN         3
#define PROBE_SWDI_PIN          PROBE_SWDIO_PIN  /* Same pin on Pico (no separate input) */
#define PROBE_SWDIR_PIN         (-1)  /* Direction control (not used on Pico) */
#define PROBE_RESET_PIN         6     /* Target reset */

/* JTAG pins for Pico (TDI/TDO available) */
#define PROBE_TCK_PIN           PROBE_SWCLK_PIN  /* TCK = SWCLK */
#define PROBE_TMS_PIN           PROBE_SWDIO_PIN  /* TMS = SWDIO */
#define PROBE_TDI_PIN           4     /* TDI output to target */
#define PROBE_TDO_PIN           5     /* TDO input from target */
#define PROBE_TRST_PIN          (-1)  /* nTRST not connected */

#define PROBE_UART_TX           0     /* UART0 TX */
#define PROBE_UART_RX           1     /* UART0 RX */
#define PROBE_UART_INTERFACE    uart0

/* LED pins for Pico */
#define PROBE_DAP_LED           25  /* On-board LED */
#define PROBE_UART_TX_LED       (-1) /* Not available */
#define PROBE_UART_RX_LED       (-1) /* Not available */

/* No level shifter on Pico */
#define PROBE_IO_OEN            (-1)

#else
/* Debug Probe Hardware configuration
 *
 * From original board_debug_probe_config.h - the Debug Probe uses:
 *   - SWCLK (GPIO12): Clock output
 *   - SWDI (GPIO13): Data INPUT from target (separate pin via level shifter)
 *   - SWDIO (GPIO14): Data OUTPUT to target (via level shifter)
 *
 * The level shifter has auto-direction sensing - no OE or direction control.
 * The input and output paths are separate, which is why SWDI != SWDIO.
 *
 * Pin offset for PIO: 12 (consecutive pins for SWCLK=12, then either SWDI or SWDIO)
 * Note: The probe.pio program (not probe_oen.pio) is used since there's no SWDIOEN.
 */
#define PROBE_PIN_OFFSET        12    /* Base pin for PIO consecutive pins */
#define PROBE_SWCLK_PIN         12    /* SWCLK output */
#define PROBE_SWDI_PIN          13    /* SWDI - Data input from target */
#define PROBE_SWDIO_PIN         14    /* SWDIO - Data output to target */
#define PROBE_SWDIR_PIN         (-1)  /* No direction control */
#define PROBE_RESET_PIN         (-1)  /* No reset pin on Debug Probe */

/* JTAG pins for Debug Probe (TDI/TDO not available on hardware) */
#define PROBE_TCK_PIN           PROBE_SWCLK_PIN  /* TCK = SWCLK */
#define PROBE_TMS_PIN           PROBE_SWDIO_PIN  /* TMS = SWDIO */
#define PROBE_TDI_PIN           (-1)  /* TDI not available */
#define PROBE_TDO_PIN           (-1)  /* TDO not available */
#define PROBE_TRST_PIN          (-1)  /* nTRST not available */

#define PROBE_UART_TX           4     /* Target UART TX (J2) */
#define PROBE_UART_RX           5     /* Target UART RX (J2) */
#define PROBE_UART_INTERFACE    uart1

/*
 * LED pins for Debug Probe hardware
 * Matching original debugprobe firmware and hardware schematic:
 *   D1 (Red, GPIO2):     USB connected indicator
 *   D2 (Green, GPIO7):   UART RX activity
 *   D3 (Yellow, GPIO8):  UART TX activity
 *   D4 (Green, GPIO15):  DAP connected
 *   D5 (Yellow, GPIO16): DAP running/activity
 */
#define PROBE_USB_CONNECTED_LED 2     /* D1 Red - USB connected */
#define PROBE_UART_RX_LED       7     /* D2 Green - UART RX activity */
#define PROBE_UART_TX_LED       8     /* D3 Yellow - UART TX activity */
#define PROBE_DAP_CONNECTED_LED 15    /* D4 Green - DAP connected */
#define PROBE_DAP_RUNNING_LED   16    /* D5 Yellow - DAP running */

/* Legacy alias for backward compatibility */
#define PROBE_DAP_LED           PROBE_DAP_RUNNING_LED

/* No level shifter output enable on Debug Probe - auto-direction sensing */
#define PROBE_IO_OEN            (-1)
#endif

/* JTAG availability check */
#define PROBE_JTAG_TDI_AVAILABLE  (PROBE_TDI_PIN >= 0)
#define PROBE_JTAG_TDO_AVAILABLE  (PROBE_TDO_PIN >= 0)
#define PROBE_JTAG_FULL_SUPPORT   (PROBE_JTAG_TDI_AVAILABLE && PROBE_JTAG_TDO_AVAILABLE)

/* USB Configuration */
#define CFG_TUD_HID_EP_BUFSIZE  64
#define DAP_PACKET_SIZE         64
/* Packet count for USB buffering.
 * Must be at least 2 for ring buffer logic to work (modulo N-1 check).
 * Value of 4 allows more pipelining depth than original debugprobe (2). */
#define DAP_PACKET_COUNT        4

/* UART Configuration */
#ifndef CONFIG_DEBUGPROBE_UART_BAUDRATE
#define PROBE_UART_BAUDRATE     115200
#else
#define PROBE_UART_BAUDRATE     CONFIG_DEBUGPROBE_UART_BAUDRATE
#endif

/* DAP Configuration */
#ifndef CONFIG_DEBUGPROBE_DAP_CLOCK_HZ
/* Default SWD clock: 10 MHz works reliably with most targets.
 * RP2040 PIO maximum: ~24 MHz (125 MHz sys_clk / 4 cycles per bit / ~1.3 margin).
 * Actual limit depends on target, wiring, and signal integrity. */
#define DAP_DEFAULT_SWJ_CLOCK   10000000
#else
#define DAP_DEFAULT_SWJ_CLOCK   CONFIG_DEBUGPROBE_DAP_CLOCK_HZ
#endif

/* CMSIS-DAP Protocol */
#define DAP_FW_VER              "2.1.0"
#define DAP_VENDOR              "Raspberry Pi"
#define DAP_PRODUCT             "Debug Probe (Zephyr)"

/* Thread Configuration */
#ifndef CONFIG_DEBUGPROBE_UART_STACK_SIZE
#define UART_THREAD_STACK_SIZE  1024
#else
#define UART_THREAD_STACK_SIZE  CONFIG_DEBUGPROBE_UART_STACK_SIZE
#endif

#ifndef CONFIG_DEBUGPROBE_DAP_STACK_SIZE
#define DAP_THREAD_STACK_SIZE   1024
#else
#define DAP_THREAD_STACK_SIZE   CONFIG_DEBUGPROBE_DAP_STACK_SIZE
#endif

#ifndef CONFIG_DEBUGPROBE_USB_STACK_SIZE
#define USB_THREAD_STACK_SIZE   1024
#else
#define USB_THREAD_STACK_SIZE   CONFIG_DEBUGPROBE_USB_STACK_SIZE
#endif

/* Thread Priorities (Zephyr: higher number = higher priority) */
#ifndef CONFIG_DEBUGPROBE_UART_THREAD_PRIORITY
#define UART_THREAD_PRIORITY    5
#else
#define UART_THREAD_PRIORITY    CONFIG_DEBUGPROBE_UART_THREAD_PRIORITY
#endif

#ifndef CONFIG_DEBUGPROBE_DAP_THREAD_PRIORITY
#define DAP_THREAD_PRIORITY     4
#else
#define DAP_THREAD_PRIORITY     CONFIG_DEBUGPROBE_DAP_THREAD_PRIORITY
#endif

#ifndef CONFIG_DEBUGPROBE_USB_THREAD_PRIORITY
#define USB_THREAD_PRIORITY     6
#else
#define USB_THREAD_PRIORITY     CONFIG_DEBUGPROBE_USB_THREAD_PRIORITY
#endif

/* Buffer sizes */
#define UART_TX_BUF_SIZE        32
#define UART_RX_BUF_SIZE        32
#define USB_TX_BUF_SIZE         256
#define USB_RX_BUF_SIZE         256

/* LED debounce timing (ms) */
#define LED_DEBOUNCE_MS         40

/* Auto-baud magic value */
#define AUTOBAUD_MAGIC          1200
