/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Device Watchdog Implementation
 *
 * Monitors USB state and system health, handles reconnection.
 * Port of dev_mon task from the original debugprobe.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include "watchdog.h"
#include "usbd_init.h"

LOG_MODULE_REGISTER(watchdog, CONFIG_LOG_DEFAULT_LEVEL);

/* Watchdog configuration */
#define WATCHDOG_POLL_INTERVAL_MS   100
#define WATCHDOG_TIMEOUT_MS         5000
#define USB_RECONNECT_DELAY_MS      1000
#define USB_SOF_SAMPLES             3

/* Watchdog state */
static struct {
    uint32_t boot_time;
    uint32_t last_feed_time;
    uint32_t error_count;
    bool running;
    /* USB SOF monitoring (for detecting stalled USB) */
    uint32_t sof_history[USB_SOF_SAMPLES];
    uint8_t sof_idx;
    uint32_t sof_stall_count;
} wdt_state;

/* USB SOF hardware access (RP2040 specific) */
#ifdef CONFIG_SOC_RP2040
#include <hardware/structs/usb.h>

static uint32_t get_usb_sof(void)
{
    return usb_hw->sof_rd & USB_SOF_RD_BITS;
}
#else
/* Stub for non-RP2040 platforms */
static uint32_t get_usb_sof(void)
{
    return 0;
}
#endif

/* Hardware watchdog (if available) */
#ifdef CONFIG_WATCHDOG
#include <zephyr/drivers/watchdog.h>
static const struct device *hw_wdt;
static int wdt_channel_id;
#endif

/* External USB state from main.c */
extern bool is_usb_configured(void);

/*
 * Check USB SOF (Start of Frame) for stall detection
 * If SOF value doesn't change while USB is active, the controller is stalled
 */
static bool check_usb_sof_stall(void)
{
#ifdef CONFIG_SOC_RP2040
    /* Only check if USB is configured */
    if (!is_usb_configured()) {
        /* Clear history when not active */
        memset(wdt_state.sof_history, 0, sizeof(wdt_state.sof_history));
        wdt_state.sof_idx = 0;
        return false;
    }

    /* Sample current SOF value */
    uint32_t sof = get_usb_sof();
    wdt_state.sof_history[wdt_state.sof_idx] = sof;
    wdt_state.sof_idx = (wdt_state.sof_idx + 1) % USB_SOF_SAMPLES;

    /* Check if all samples are identical (stalled) */
    bool all_same = true;
    for (int i = 1; i < USB_SOF_SAMPLES; i++) {
        if (wdt_state.sof_history[i] != wdt_state.sof_history[0]) {
            all_same = false;
            break;
        }
    }

    /* If all samples are the same and non-zero, USB is stalled */
    if (all_same && wdt_state.sof_history[0] != 0) {
        wdt_state.sof_stall_count++;
        return true;
    }

    wdt_state.sof_stall_count = 0;
    return false;
#else
    return false;
#endif
}

/*
 * Reset USB controller (for recovery from stall)
 */
static void reset_usb_controller(void)
{
    struct usbd_context *ctx = debugprobe_usbd_get_context();

    if (ctx == NULL) {
        LOG_ERR("USB context not available");
        wdt_state.error_count++;
        return;
    }

    LOG_WRN("USB controller stall detected - resetting");

    int ret = usbd_disable(ctx);
    if (ret == 0) {
        k_sleep(K_MSEC(100));
        ret = usbd_enable(ctx);
        if (ret == 0) {
            LOG_INF("USB controller reset successful");
        } else {
            LOG_ERR("Failed to re-enable USB: %d", ret);
            wdt_state.error_count++;
        }
    } else {
        LOG_ERR("Failed to disable USB: %d", ret);
        wdt_state.error_count++;
    }

    /* Clear SOF history after reset */
    memset(wdt_state.sof_history, 0, sizeof(wdt_state.sof_history));
    wdt_state.sof_idx = 0;
    wdt_state.sof_stall_count = 0;
}

/*
 * Check and handle USB state
 */
static void check_usb_state(void)
{
    static bool prev_configured = false;
    static uint32_t disconnect_time = 0;
    bool usb_configured = is_usb_configured();

    /* Check for USB SOF stall */
    if (check_usb_sof_stall()) {
        if (wdt_state.sof_stall_count >= 3) {
            /* Stalled for 3 consecutive checks (~300ms) */
            reset_usb_controller();
        }
    }

    if (!usb_configured && prev_configured) {
        /* Just disconnected */
        disconnect_time = k_uptime_get_32();
        LOG_INF("USB disconnected, monitoring for reconnection");
    }

    if (!usb_configured && disconnect_time > 0) {
        uint32_t elapsed = k_uptime_get_32() - disconnect_time;

        /* Check if we should attempt reconnection */
        if (elapsed > USB_RECONNECT_DELAY_MS) {
            LOG_INF("Attempting USB reconnection");
            reset_usb_controller();
            disconnect_time = 0;  /* Reset timer */
        }
    }

    prev_configured = usb_configured;
}

/*
 * Check software watchdog timeout
 */
static void check_software_timeout(void)
{
    uint32_t now = k_uptime_get_32();
    uint32_t elapsed = now - wdt_state.last_feed_time;

    if (elapsed > WATCHDOG_TIMEOUT_MS && wdt_state.last_feed_time > 0) {
        LOG_WRN("Software watchdog timeout (%u ms)", elapsed);
        wdt_state.error_count++;
        /* Don't reset on software timeout - just log */
        wdt_state.last_feed_time = now;  /* Reset timer */
    }
}

/*
 * Feed hardware watchdog (if available)
 */
static void feed_hw_watchdog(void)
{
#ifdef CONFIG_WATCHDOG
    if (hw_wdt != NULL) {
        wdt_feed(hw_wdt, wdt_channel_id);
    }
#endif
}

/*
 * Watchdog monitoring thread
 */
static void watchdog_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Watchdog thread started");

    while (wdt_state.running) {
        /* Check USB state */
        check_usb_state();

        /* Check software watchdog */
        check_software_timeout();

        /* Feed hardware watchdog */
        feed_hw_watchdog();

        /* Sleep for poll interval */
        k_sleep(K_MSEC(WATCHDOG_POLL_INTERVAL_MS));
    }

    LOG_INF("Watchdog thread stopped");
}

K_THREAD_STACK_DEFINE(watchdog_stack, 1024);
static struct k_thread watchdog_thread;

/*
 * Initialize device watchdog
 */
int watchdog_init(void)
{
    /* Initialize state */
    memset(&wdt_state, 0, sizeof(wdt_state));
    wdt_state.boot_time = k_uptime_get_32();
    wdt_state.running = true;

#ifdef CONFIG_WATCHDOG
    /* Initialize hardware watchdog if available */
    hw_wdt = DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0));
    if (hw_wdt != NULL && device_is_ready(hw_wdt)) {
        struct wdt_timeout_cfg wdt_config = {
            .window.min = 0,
            .window.max = 10000,  /* 10 second timeout */
            .callback = NULL,
            .flags = WDT_FLAG_RESET_SOC,
        };

        wdt_channel_id = wdt_install_timeout(hw_wdt, &wdt_config);
        if (wdt_channel_id >= 0) {
            int ret = wdt_setup(hw_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
            if (ret == 0) {
                LOG_INF("Hardware watchdog enabled");
            } else {
                LOG_WRN("Failed to setup hardware watchdog: %d", ret);
                hw_wdt = NULL;
            }
        } else {
            LOG_WRN("Failed to install watchdog timeout: %d", wdt_channel_id);
            hw_wdt = NULL;
        }
    } else {
        LOG_DBG("Hardware watchdog not available");
        hw_wdt = NULL;
    }
#endif

    /* Start watchdog thread */
    k_thread_create(&watchdog_thread, watchdog_stack,
                    K_THREAD_STACK_SIZEOF(watchdog_stack),
                    watchdog_thread_entry, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

    LOG_INF("Device watchdog initialized");

    return 0;
}

/*
 * Feed the watchdog
 */
void watchdog_feed(void)
{
    wdt_state.last_feed_time = k_uptime_get_32();
}

/*
 * Check if USB is connected
 */
bool watchdog_usb_connected(void)
{
    return is_usb_configured();
}

/*
 * Get uptime in milliseconds
 */
uint32_t watchdog_get_uptime_ms(void)
{
    return k_uptime_get_32() - wdt_state.boot_time;
}

/*
 * Get error count
 */
uint32_t watchdog_get_error_count(void)
{
    return wdt_state.error_count;
}

/*
 * Trigger system reset
 */
void watchdog_reset(void)
{
    LOG_WRN("System reset requested");

    /* Disable interrupts and reset */
    sys_reboot(SYS_REBOOT_COLD);
}
