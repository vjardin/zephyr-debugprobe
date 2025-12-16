/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - SWO (Serial Wire Output) Implementation
 *
 * SWO captures trace data from the target's TRACESWO pin.
 * Supports UART encoding mode (Manchester is more complex).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "probe_config.h"
#include "swo.h"

LOG_MODULE_REGISTER(swo, CONFIG_LOG_DEFAULT_LEVEL);

/* SWO buffer size */
#ifndef CONFIG_DEBUGPROBE_SWO_BUFFER_SIZE
#define SWO_BUFFER_SIZE 4096
#else
#define SWO_BUFFER_SIZE CONFIG_DEBUGPROBE_SWO_BUFFER_SIZE
#endif

/* SWO ring buffer for captured data */
RING_BUF_DECLARE(swo_ringbuf, SWO_BUFFER_SIZE);

/* SWO state */
static uint8_t swo_transport = SWO_TRANSPORT_NONE;
static uint8_t swo_mode = SWO_MODE_OFF;
static uint32_t swo_baudrate = 0;
static bool swo_active = false;
static bool swo_error = false;
static bool swo_overrun = false;

/* SWO UART device (if using UART mode)
 *
 * Note: SWO is NOT implemented in the original debugprobe hardware
 * (SWO_UART=0, SWO_MANCHESTER=0 in DAP_config.h).
 * This module provides the framework for future SWO support.
 * To enable SWO, define CONFIG_DEBUGPROBE_SWO and set
 * PROBE_SWO_PIN in probe_config.h for your board.
 */
static const struct device *swo_uart_dev;

#ifdef CONFIG_DEBUGPROBE_SWO
/*
 * UART IRQ callback for SWO capture
 */
static void swo_uart_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t c;
            while (uart_fifo_read(dev, &c, 1) == 1) {
                if (ring_buf_put(&swo_ringbuf, &c, 1) != 1) {
                    swo_overrun = true;
                }
            }
        }
    }
}
#endif /* CONFIG_DEBUGPROBE_SWO */

/*
 * Initialize SWO capture
 */
int swo_init(void)
{
    swo_transport = SWO_TRANSPORT_NONE;
    swo_mode = SWO_MODE_OFF;
    swo_baudrate = 0;
    swo_active = false;
    swo_error = false;
    swo_overrun = false;

    ring_buf_reset(&swo_ringbuf);

#ifdef CONFIG_DEBUGPROBE_SWO
    /* Initialize SWO UART device
     * Note: Requires PROBE_SWO_UART to be defined in probe_config.h
     * For example: #define PROBE_SWO_UART uart0
     */
#ifdef PROBE_SWO_UART
    swo_uart_dev = DEVICE_DT_GET(DT_NODELABEL(PROBE_SWO_UART));
    if (!device_is_ready(swo_uart_dev)) {
        LOG_WRN("SWO UART device not ready");
        swo_uart_dev = NULL;
    } else {
        /* Register IRQ callback but don't enable yet */
        uart_irq_callback_set(swo_uart_dev, swo_uart_irq_callback);
        LOG_INF("SWO UART initialized");
    }
#else
    LOG_WRN("SWO enabled but PROBE_SWO_UART not defined");
#endif /* PROBE_SWO_UART */
#endif /* CONFIG_DEBUGPROBE_SWO */

    LOG_INF("SWO initialized");
    return 0;
}

/*
 * Set SWO transport mode
 */
int swo_set_transport(uint8_t transport)
{
    if (transport > SWO_TRANSPORT_WINUSB) {
        return -EINVAL;
    }

    swo_transport = transport;
    LOG_DBG("SWO transport set to %d", transport);
    return 0;
}

/*
 * Get current SWO transport mode
 */
uint8_t swo_get_transport(void)
{
    return swo_transport;
}

/*
 * Set SWO mode
 */
int swo_set_mode(uint8_t mode)
{
    if (mode > SWO_MODE_MANCHESTER) {
        return -EINVAL;
    }

    /* Stop capture if changing mode */
    if (swo_active && mode != swo_mode) {
        swo_stop();
    }

    swo_mode = mode;
    ring_buf_reset(&swo_ringbuf);
    swo_error = false;
    swo_overrun = false;

    LOG_DBG("SWO mode set to %d", mode);
    return 0;
}

/*
 * Get current SWO mode
 */
uint8_t swo_get_mode(void)
{
    return swo_mode;
}

/*
 * Set SWO baudrate
 */
uint32_t swo_set_baudrate(uint32_t baudrate)
{
    if (baudrate == 0) {
        return 0;
    }

    swo_baudrate = baudrate;

    /* If using UART mode and device is available, configure it */
    if (swo_mode == SWO_MODE_UART && swo_uart_dev != NULL) {
        struct uart_config cfg;

        if (uart_config_get(swo_uart_dev, &cfg) == 0) {
            cfg.baudrate = baudrate;
            cfg.parity = UART_CFG_PARITY_NONE;
            cfg.stop_bits = UART_CFG_STOP_BITS_1;
            cfg.data_bits = UART_CFG_DATA_BITS_8;
            cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

            if (uart_configure(swo_uart_dev, &cfg) == 0) {
                LOG_INF("SWO baudrate set to %u", baudrate);
            }
        }
    }

    return swo_baudrate;
}

/*
 * Get current SWO baudrate
 */
uint32_t swo_get_baudrate(void)
{
    return swo_baudrate;
}

/*
 * Start SWO capture
 */
int swo_start(void)
{
    if (swo_mode == SWO_MODE_OFF) {
        return -EINVAL;
    }

    ring_buf_reset(&swo_ringbuf);
    swo_error = false;
    swo_overrun = false;
    swo_active = true;

    if (swo_mode == SWO_MODE_UART && swo_uart_dev != NULL) {
        uart_irq_rx_enable(swo_uart_dev);
    }

    LOG_INF("SWO capture started");
    return 0;
}

/*
 * Stop SWO capture
 */
int swo_stop(void)
{
    swo_active = false;

    if (swo_uart_dev != NULL) {
        uart_irq_rx_disable(swo_uart_dev);
    }

    LOG_INF("SWO capture stopped");
    return 0;
}

/*
 * Get SWO status
 */
uint8_t swo_get_status(void)
{
    uint8_t status = 0;

    if (swo_active) {
        status |= SWO_STATUS_CAPTURE;
    }
    if (swo_error) {
        status |= SWO_STATUS_ERROR;
    }
    if (swo_overrun) {
        status |= SWO_STATUS_OVERRUN;
    }

    return status;
}

/*
 * Get SWO trace count
 */
uint32_t swo_get_count(void)
{
    return ring_buf_size_get(&swo_ringbuf);
}

/*
 * Read SWO trace data
 */
uint32_t swo_read_data(uint8_t *buffer, uint32_t max_len)
{
    return ring_buf_get(&swo_ringbuf, buffer, max_len);
}

/*
 * Get SWO buffer size
 */
uint32_t swo_get_buffer_size(void)
{
    return SWO_BUFFER_SIZE;
}
