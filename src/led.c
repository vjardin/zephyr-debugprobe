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

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "led.h"
#include "probe_config.h"

LOG_MODULE_REGISTER(led, CONFIG_LOG_DEFAULT_LEVEL);

/* LED GPIO specifications from devicetree */
#if DT_NODE_EXISTS(DT_NODELABEL(red_led))
static const struct gpio_dt_spec led_usb = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led), gpios);
#define HAS_USB_LED 1
#else
#define HAS_USB_LED 0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(green_led_uart))
static const struct gpio_dt_spec led_rx = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_uart), gpios);
#define HAS_RX_LED 1
#else
#define HAS_RX_LED 0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(yellow_led_uart))
static const struct gpio_dt_spec led_tx = GPIO_DT_SPEC_GET(DT_NODELABEL(yellow_led_uart), gpios);
#define HAS_TX_LED 1
#else
#define HAS_TX_LED 0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(green_led_dbug))
static const struct gpio_dt_spec led_dap_conn = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_dbug), gpios);
#define HAS_DAP_CONNECTED_LED 1
#else
#define HAS_DAP_CONNECTED_LED 0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(yellow_led_dbug))
static const struct gpio_dt_spec led_dap_run = GPIO_DT_SPEC_GET(DT_NODELABEL(yellow_led_dbug), gpios);
#define HAS_DAP_RUNNING_LED 1
#else
#define HAS_DAP_RUNNING_LED 0
#endif

/* Debounce state for activity LEDs */
static volatile uint32_t tx_debounce_count;
static volatile uint32_t rx_debounce_count;
static uint32_t debounce_ticks = LED_DEBOUNCE_MS;

/* SWD error blink state (uses timestamps for timing) */
#define SWD_ERROR_BLINK_MS     100   /* Blink toggle period */
#define SWD_ERROR_TIMEOUT_MS   10000 /* Continue blinking 10s after last error */
static volatile int64_t swd_error_last_failure;  /* Time of last failure */
static volatile int64_t swd_error_last_toggle;
static volatile bool swd_error_active;
static volatile bool swd_error_led_state;

/* LED initialization flag */
static bool leds_initialized = false;

int led_init(void)
{
    int ret;

#if HAS_USB_LED
    if (!gpio_is_ready_dt(&led_usb)) {
        LOG_WRN("USB LED GPIO not ready");
    } else {
        ret = gpio_pin_configure_dt(&led_usb, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure USB LED: %d", ret);
        } else {
            LOG_DBG("USB LED (D1) configured on GPIO%d", led_usb.pin);
        }
    }
#endif

#if HAS_RX_LED
    if (!gpio_is_ready_dt(&led_rx)) {
        LOG_WRN("RX LED GPIO not ready");
    } else {
        ret = gpio_pin_configure_dt(&led_rx, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure RX LED: %d", ret);
        } else {
            LOG_DBG("RX LED (D2) configured on GPIO%d", led_rx.pin);
        }
    }
#endif

#if HAS_TX_LED
    if (!gpio_is_ready_dt(&led_tx)) {
        LOG_WRN("TX LED GPIO not ready");
    } else {
        ret = gpio_pin_configure_dt(&led_tx, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure TX LED: %d", ret);
        } else {
            LOG_DBG("TX LED (D3) configured on GPIO%d", led_tx.pin);
        }
    }
#endif

#if HAS_DAP_CONNECTED_LED
    if (!gpio_is_ready_dt(&led_dap_conn)) {
        LOG_WRN("DAP connected LED GPIO not ready");
    } else {
        ret = gpio_pin_configure_dt(&led_dap_conn, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure DAP connected LED: %d", ret);
        } else {
            LOG_DBG("DAP connected LED (D4) configured on GPIO%d", led_dap_conn.pin);
        }
    }
#endif

#if HAS_DAP_RUNNING_LED
    if (!gpio_is_ready_dt(&led_dap_run)) {
        LOG_WRN("DAP running LED GPIO not ready");
    } else {
        ret = gpio_pin_configure_dt(&led_dap_run, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure DAP running LED: %d", ret);
        } else {
            LOG_DBG("DAP running LED (D5) configured on GPIO%d", led_dap_run.pin);
        }
    }
#endif

    tx_debounce_count = 0;
    rx_debounce_count = 0;
    leds_initialized = true;

    LOG_INF("LEDs initialized");
    return 0;
}

void led_usb_connected(bool connected)
{
#if HAS_USB_LED
    if (leds_initialized && gpio_is_ready_dt(&led_usb)) {
        gpio_pin_set_dt(&led_usb, connected ? 1 : 0);
    }
#endif
}

void led_dap_connected(bool connected)
{
#if HAS_DAP_CONNECTED_LED
    if (leds_initialized && gpio_is_ready_dt(&led_dap_conn)) {
        gpio_pin_set_dt(&led_dap_conn, connected ? 1 : 0);
    }
#endif
}

void led_dap_running(bool running)
{
#if HAS_DAP_RUNNING_LED
    if (leds_initialized && gpio_is_ready_dt(&led_dap_run)) {
        gpio_pin_set_dt(&led_dap_run, running ? 1 : 0);
    }
#endif
}

void led_uart_tx_activity(void)
{
#if HAS_TX_LED
    if (leds_initialized && gpio_is_ready_dt(&led_tx)) {
        gpio_pin_set_dt(&led_tx, 1);
        tx_debounce_count = debounce_ticks;
    }
#endif
}

void led_uart_rx_activity(void)
{
#if HAS_RX_LED
    if (leds_initialized && gpio_is_ready_dt(&led_rx)) {
        gpio_pin_set_dt(&led_rx, 1);
        rx_debounce_count = debounce_ticks;
    }
#endif
}

void led_task(void)
{
    if (!leds_initialized) {
        return;
    }

#if HAS_TX_LED
    if (tx_debounce_count > 0) {
        tx_debounce_count--;
        if (tx_debounce_count == 0 && gpio_is_ready_dt(&led_tx)) {
            gpio_pin_set_dt(&led_tx, 0);
        }
    }
#endif

#if HAS_RX_LED
    if (rx_debounce_count > 0) {
        rx_debounce_count--;
        if (rx_debounce_count == 0 && gpio_is_ready_dt(&led_rx)) {
            gpio_pin_set_dt(&led_rx, 0);
        }
    }
#endif

#if HAS_USB_LED
    /* SWD error blinking on RED LED (timestamp-based for irregular calling) */
    if (swd_error_active) {
        int64_t now = k_uptime_get();
        int64_t since_last_failure = now - swd_error_last_failure;

        /* Check if timeout expired (no failures for 10s) */
        if (since_last_failure >= SWD_ERROR_TIMEOUT_MS) {
            swd_error_active = false;
            swd_error_led_state = false;
            gpio_pin_set_dt(&led_usb, 1);  /* Restore to ON (USB connected) */
            LOG_INF("SWD error cleared after timeout");
        } else if ((now - swd_error_last_toggle) >= SWD_ERROR_BLINK_MS) {
            /* Toggle LED */
            swd_error_led_state = !swd_error_led_state;
            gpio_pin_set_dt(&led_usb, swd_error_led_state ? 1 : 0);
            swd_error_last_toggle = now;
        }
    }
#endif
}

void led_set_debounce(uint32_t ticks_ms)
{
    debounce_ticks = ticks_ms;
}

void led_swd_error(void)
{
#if HAS_USB_LED
    if (leds_initialized && gpio_is_ready_dt(&led_usb)) {
        int64_t now = k_uptime_get();
        swd_error_last_failure = now;  /* Reset/extend timeout */

        if (!swd_error_active) {
            /* First error - start blinking */
            swd_error_last_toggle = now;
            swd_error_active = true;
            swd_error_led_state = false;
            gpio_pin_set_dt(&led_usb, 0);  /* Start with LED OFF (blink) */
            LOG_WRN("SWD error - check wiring");
        }
        /* If already active, just extends timeout via swd_error_last_failure */
    }
#endif
}

void led_swd_error_clear(void)
{
#if HAS_USB_LED
    if (leds_initialized && swd_error_active) {
        swd_error_active = false;
        swd_error_led_state = false;
        /* Restore USB connected state (LED should be on if USB is connected) */
        gpio_pin_set_dt(&led_usb, 1);
    }
#endif
}
