/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - CDC UART Bridge
 *
 * Port of cdc_uart.c from FreeRTOS to Zephyr RTOS
 * Original: https://github.com/raspberrypi/debugprobe/blob/master/src/cdc_uart.c
 *
 * This module bridges USB CDC ACM to a hardware UART.
 *
 * FreeRTOS to Zephyr mapping:
 * - TickType_t -> int64_t (k_uptime_get())
 * - xTaskGetTickCount() -> k_uptime_get_32()
 * - vTaskDelayUntil() -> k_sleep() with timing
 * - TaskHandle_t -> k_tid_t
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <string.h>

#include "probe_config.h"
#include "cdc_uart.h"
#include "led.h"
#include "autobaud.h"

/* USB CDC line control for break (from usb_cdc.h) */
#ifndef USB_CDC_LINE_CTRL_BREAK
#define USB_CDC_LINE_CTRL_BREAK     BIT(5)
#endif

LOG_MODULE_REGISTER(cdc_uart, CONFIG_LOG_DEFAULT_LEVEL);

/* UART device */
static const struct device *uart_dev;

/* CDC ACM device */
static const struct device *cdc_dev;

/* Ring buffers for async operation */
RING_BUF_DECLARE(cdc_to_uart_rb, USB_TX_BUF_SIZE);
RING_BUF_DECLARE(uart_to_cdc_rb, USB_RX_BUF_SIZE);

/* Break timing */
static volatile int64_t break_expiry;
static volatile bool timed_break;

/* Current baud rate */
static uint32_t current_baudrate = PROBE_UART_BAUDRATE;

/*
 * Software Flow Control
 * Uses buffer fill levels to apply backpressure
 */
#define FLOW_CTRL_HIGH_WATER  ((USB_RX_BUF_SIZE * 3) / 4)  /* 75% full - pause */
#define FLOW_CTRL_LOW_WATER   (USB_RX_BUF_SIZE / 4)        /* 25% full - resume */

/* Flow control state */
static volatile bool rx_flow_paused = false;
static volatile bool tx_flow_paused = false;

/* Statistics counters */
static volatile uint32_t stats_tx_bytes = 0;  /* Host -> Target (CDC RX -> UART TX) */
static volatile uint32_t stats_rx_bytes = 0;  /* Target -> Host (UART RX -> CDC TX) */
static volatile uint32_t stats_tx_overrun = 0; /* Bytes dropped due to full buffer */
static volatile uint32_t stats_rx_overrun = 0;

/*
 * Check and update flow control state
 */
static void update_flow_control(void)
{
    uint32_t uart_to_cdc_level = ring_buf_size_get(&uart_to_cdc_rb);
    uint32_t cdc_to_uart_level = ring_buf_size_get(&cdc_to_uart_rb);

    /* RX flow control: pause UART RX if buffer is getting full */
    if (!rx_flow_paused && uart_to_cdc_level >= FLOW_CTRL_HIGH_WATER) {
        rx_flow_paused = true;
        uart_irq_rx_disable(uart_dev);
        LOG_DBG("RX flow: paused (buffer %u/%u)", uart_to_cdc_level, USB_RX_BUF_SIZE);
    } else if (rx_flow_paused && uart_to_cdc_level <= FLOW_CTRL_LOW_WATER) {
        rx_flow_paused = false;
        uart_irq_rx_enable(uart_dev);
        LOG_DBG("RX flow: resumed (buffer %u/%u)", uart_to_cdc_level, USB_RX_BUF_SIZE);
    }

    /* TX flow control: pause CDC to UART if buffer is getting full */
    if (!tx_flow_paused && cdc_to_uart_level >= FLOW_CTRL_HIGH_WATER) {
        tx_flow_paused = true;
        LOG_DBG("TX flow: paused (buffer %u/%u)", cdc_to_uart_level, USB_TX_BUF_SIZE);
    } else if (tx_flow_paused && cdc_to_uart_level <= FLOW_CTRL_LOW_WATER) {
        tx_flow_paused = false;
        LOG_DBG("TX flow: resumed (buffer %u/%u)", cdc_to_uart_level, USB_TX_BUF_SIZE);
    }
}

/*
 * UART Interrupt Callback (hardware UART to target)
 */
static void uart_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        /* Handle RX - check for buffer space before reading */
        if (uart_irq_rx_ready(dev)) {
            uint8_t c;
            uint32_t free_space = ring_buf_space_get(&uart_to_cdc_rb);
            uint32_t count = 0;

            while (uart_fifo_read(dev, &c, 1) == 1) {
                if (free_space > 0) {
                    ring_buf_put(&uart_to_cdc_rb, &c, 1);
                    free_space--;
                    count++;
                } else {
                    stats_rx_overrun++;
                }
            }

            if (count > 0) {
                stats_rx_bytes += count;
                led_uart_rx_activity();
            }

            /* Apply flow control if buffer is getting full */
            if (ring_buf_size_get(&uart_to_cdc_rb) >= FLOW_CTRL_HIGH_WATER) {
                rx_flow_paused = true;
                uart_irq_rx_disable(dev);
            }
        }

        /* Handle TX */
        if (uart_irq_tx_ready(dev)) {
            uint8_t c;
            if (ring_buf_get(&cdc_to_uart_rb, &c, 1) == 1) {
                uart_fifo_fill(dev, &c, 1);
                led_uart_tx_activity();
            } else {
                uart_irq_tx_disable(dev);
            }
        }
    }
}

/*
 * CDC ACM Interrupt Callback (USB host to probe)
 * Handles data received from the USB host on the UART bridge interface
 */
static void cdc_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        /* Handle RX - data from USB host to be sent to target UART */
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            int len;

            len = uart_fifo_read(dev, buf, sizeof(buf));
            if (len > 0) {
                uint32_t free_space = ring_buf_space_get(&cdc_to_uart_rb);
                uint32_t to_write = (free_space >= len) ? len : free_space;

                if (to_write > 0) {
                    ring_buf_put(&cdc_to_uart_rb, buf, to_write);
                    stats_tx_bytes += to_write;
                }
                if (to_write < len) {
                    stats_tx_overrun += (len - to_write);
                    tx_flow_paused = true;
                }
                led_uart_tx_activity();
                /* Trigger UART TX */
                uart_irq_tx_enable(uart_dev);
            }
        }

        /* Handle TX - data from target UART to be sent to USB host */
        if (uart_irq_tx_ready(dev)) {
            uint8_t buf[64];
            uint32_t len = ring_buf_get(&uart_to_cdc_rb, buf, sizeof(buf));
            if (len > 0) {
                int sent = uart_fifo_fill(dev, buf, len);
                if (sent < len) {
                    /* Put back unsent bytes */
                    ring_buf_put(&uart_to_cdc_rb, &buf[sent], len - sent);
                }
            } else {
                uart_irq_tx_disable(dev);
            }
        }
    }
}

/*
 * Poll CDC ACM for baud rate changes
 * The new USB stack signals line coding changes via messages,
 * and we read the new values via uart_line_ctrl_get.
 */
static void cdc_acm_poll_baudrate(void)
{
    uint32_t rate = 0;
    struct uart_config cfg;

    if (!cdc_dev) {
        return;
    }

    /* Get current baud rate from CDC ACM */
    if (uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_BAUD_RATE, &rate) != 0) {
        return;
    }

    /* Check if rate changed */
    if (rate == current_baudrate || rate == 0) {
        return;
    }

    LOG_INF("CDC baud rate changed: %u -> %u", current_baudrate, rate);

#if defined(CONFIG_DEBUGPROBE_AUTOBAUD) && defined(CONFIG_SOC_RP2040)
    /* Check for autobaud magic value */
    if (rate == AUTOBAUD_MAGIC) {
        LOG_INF("Autobaud mode requested");
        /* Stop any running detection first */
        if (autobaud_is_running()) {
            autobaud_stop();
        }
        /* Start autobaud detection on UART RX pin */
        int ret = autobaud_start(PROBE_UART_RX);
        if (ret < 0) {
            LOG_ERR("Failed to start autobaud: %d", ret);
        }
        current_baudrate = rate;
        return;
    }
#endif

    /* Update target UART baud rate */
    if (uart_config_get(uart_dev, &cfg) == 0) {
        cfg.baudrate = rate;
        if (uart_configure(uart_dev, &cfg) == 0) {
            current_baudrate = rate;
            LOG_INF("Target UART configured to %u baud", rate);
        } else {
            LOG_ERR("Failed to configure target UART");
        }
    }
}

/*
 * Poll CDC ACM Control Line State
 * Check DTR/RTS status - polled since legacy stack has no callback
 */
static void cdc_acm_poll_control_lines(void)
{
    static bool last_dtr = false;
    static bool last_rts = false;
    uint32_t dtr_val = 0;
    uint32_t rts_val = 0;

    if (!cdc_dev) {
        return;
    }

    uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr_val);
    uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_RTS, &rts_val);

    bool dtr = dtr_val != 0;
    bool rts = rts_val != 0;

    if (dtr != last_dtr || rts != last_rts) {
        LOG_DBG("Control lines changed: DTR=%d RTS=%d", dtr, rts);
        last_dtr = dtr;
        last_rts = rts;
        /* Could use DTR/RTS for flow control or target reset */
    }
}

/*
 * Handle break condition
 * Check for break request via line control (polled)
 */
static void cdc_acm_handle_break(void)
{
    static bool last_break = false;
    uint32_t break_val = 0;

    if (!cdc_dev) {
        return;
    }

    /* Check break status via line control */
    if (uart_line_ctrl_get(cdc_dev, USB_CDC_LINE_CTRL_BREAK, &break_val) == 0) {
        bool break_active = break_val != 0;

        if (break_active != last_break) {
            LOG_INF("Break state: %s", break_active ? "active" : "inactive");
            last_break = break_active;

            if (break_active) {
                /* Start break - set expiry if timed */
                timed_break = true;
                break_expiry = k_uptime_get() + 250;  /* Default 250ms */
            } else {
                /* End break */
                timed_break = false;
            }
        }
    }
}

/*
 * Initialize CDC UART Bridge
 * Replaces cdc_uart_init() from original
 */
void cdc_uart_init(void)
{
    int ret;
    struct uart_config cfg;

    LOG_INF("Initializing CDC UART bridge");

    /* Get UART device */
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return;
    }

    /* Configure UART */
    cfg.baudrate = PROBE_UART_BAUDRATE;
    cfg.parity = UART_CFG_PARITY_NONE;
    cfg.stop_bits = UART_CFG_STOP_BITS_1;
    cfg.data_bits = UART_CFG_DATA_BITS_8;
    cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    ret = uart_configure(uart_dev, &cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        return;
    }

    /* Set up UART interrupt callback */
    uart_irq_callback_set(uart_dev, uart_irq_callback);
    uart_irq_rx_enable(uart_dev);

    /* Get CDC ACM device #1 for UART bridge (device #0 is console) */
    cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
    if (!device_is_ready(cdc_dev)) {
        LOG_ERR("CDC ACM UART bridge device not ready");
        return;
    }

    /* Set up CDC ACM interrupt callback for receiving data from USB host */
    uart_irq_callback_set(cdc_dev, cdc_irq_callback);
    uart_irq_rx_enable(cdc_dev);

    /* Note: Baud rate changes are now handled via polling in cdc_uart_task() */
    /* LED initialization is handled by led_init() in main.c */

    LOG_INF("CDC UART bridge initialized at %u baud", PROBE_UART_BAUDRATE);
}

/*
 * CDC UART Task
 * Replaces the main loop of cdc_thread()
 *
 * Original FreeRTOS pattern:
 *   vTaskDelayUntil(&last_wake, interval);
 *   [process data]
 *
 * Zephyr equivalent:
 *   k_sleep(K_USEC(interval));
 *   [process data]
 */
void cdc_uart_task(void)
{
    /* Handle timed break expiry */
    if (timed_break && k_uptime_get() >= break_expiry) {
        timed_break = false;
        LOG_DBG("Break expired");
    }

#if defined(CONFIG_DEBUGPROBE_AUTOBAUD) && defined(CONFIG_SOC_RP2040)
    /* Check for autobaud result */
    if (autobaud_is_running()) {
        struct autobaud_result result;
        if (autobaud_get_result(&result)) {
            /* Autobaud detected - apply the baud rate */
            LOG_INF("Autobaud complete: %u baud (validity: %.2f)",
                    result.baud, (double)result.validity);
            autobaud_stop();

            /* Update UART configuration */
            struct uart_config cfg;
            if (uart_config_get(uart_dev, &cfg) == 0) {
                cfg.baudrate = result.baud;
                if (uart_configure(uart_dev, &cfg) == 0) {
                    current_baudrate = result.baud;
                    LOG_INF("UART configured to detected %u baud", result.baud);
                }
            }
        }
    }
#endif

    /* Update flow control state */
    update_flow_control();

    /* CDC RX is now handled by cdc_irq_callback() */

    /* Trigger UART TX if there's data in the cdc_to_uart ring buffer */
    if (!ring_buf_is_empty(&cdc_to_uart_rb)) {
        uart_irq_tx_enable(uart_dev);
    }

    /* Trigger CDC TX if there's data in the uart_to_cdc ring buffer */
    if (!ring_buf_is_empty(&uart_to_cdc_rb) && cdc_dev) {
        uart_irq_tx_enable(cdc_dev);
    }

    /* Resume UART RX if we drained the buffer below threshold */
    if (rx_flow_paused &&
        ring_buf_size_get(&uart_to_cdc_rb) <= FLOW_CTRL_LOW_WATER) {
        rx_flow_paused = false;
        uart_irq_rx_enable(uart_dev);
        LOG_DBG("RX flow: resumed after CDC drain");
    }

    /* Poll for baud rate changes */
    cdc_acm_poll_baudrate();

    /* Poll for control line changes (DTR/RTS) */
    cdc_acm_poll_control_lines();

    /* Handle break condition */
    cdc_acm_handle_break();

    /* Update LED debounce timers */
    led_task();
}

/*
 * Write data from CDC to UART
 * Called when USB CDC receives data from host
 * Returns number of bytes actually written (may be less if flow controlled)
 */
void cdc_uart_write(const uint8_t *data, size_t len)
{
    /* Check flow control - limit write if buffer is getting full */
    uint32_t free_space = ring_buf_space_get(&cdc_to_uart_rb);
    if (free_space < len) {
        len = free_space;
        tx_flow_paused = true;
        LOG_DBG("TX flow: backpressure applied (free: %u)", free_space);
    }

    if (len > 0) {
        ring_buf_put(&cdc_to_uart_rb, data, len);
        uart_irq_tx_enable(uart_dev);
    }
}

/*
 * Check if CDC to UART is flow controlled
 */
bool cdc_uart_tx_ready(void)
{
    return !tx_flow_paused && ring_buf_space_get(&cdc_to_uart_rb) > 0;
}

/*
 * Read data from UART for CDC
 * Returns number of bytes read
 */
size_t cdc_uart_read(uint8_t *data, size_t max_len)
{
    return ring_buf_get(&uart_to_cdc_rb, data, max_len);
}

/*
 * Check if there's data available from UART
 */
bool cdc_uart_data_available(void)
{
    return !ring_buf_is_empty(&uart_to_cdc_rb);
}

/*
 * Shell Commands for UART bridge diagnostics
 */

/* uart stats - show byte counters and buffer status */
static int cmd_uart_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "UART Bridge Statistics:");
    shell_print(sh, "  TX (Host->Target): %u bytes, %u overrun",
                stats_tx_bytes, stats_tx_overrun);
    shell_print(sh, "  RX (Target->Host): %u bytes, %u overrun",
                stats_rx_bytes, stats_rx_overrun);
    shell_print(sh, "Buffer Status:");
    shell_print(sh, "  cdc_to_uart: %u/%u bytes",
                ring_buf_size_get(&cdc_to_uart_rb), USB_TX_BUF_SIZE);
    shell_print(sh, "  uart_to_cdc: %u/%u bytes",
                ring_buf_size_get(&uart_to_cdc_rb), USB_RX_BUF_SIZE);
    shell_print(sh, "Flow Control:");
    shell_print(sh, "  TX paused: %s, RX paused: %s",
                tx_flow_paused ? "yes" : "no",
                rx_flow_paused ? "yes" : "no");
    shell_print(sh, "Baud Rate: %u", current_baudrate);

    return 0;
}

/* uart reset - reset statistics counters */
static int cmd_uart_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    stats_tx_bytes = 0;
    stats_rx_bytes = 0;
    stats_tx_overrun = 0;
    stats_rx_overrun = 0;

    shell_print(sh, "Statistics reset");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart,
    SHELL_CMD(stats, NULL, "Show UART bridge statistics", cmd_uart_stats),
    SHELL_CMD(reset, NULL, "Reset statistics counters", cmd_uart_reset),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(uart, &sub_uart, "UART bridge commands", NULL);
