/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Main Entry Point
 *
 * Port of Raspberry Pi Debug Probe firmware from FreeRTOS to Zephyr RTOS
 * Original: https://github.com/raspberrypi/debugprobe
 *
 * This file replaces the FreeRTOS-based main.c with Zephyr equivalents:
 * - xTaskCreate() -> K_THREAD_DEFINE() or k_thread_create()
 * - vTaskDelay() -> k_sleep()
 * - vTaskDelayUntil() -> k_sleep() with timing calculation
 * - TaskHandle_t -> k_tid_t
 * - Queues -> k_msgq
 * - Semaphores -> k_sem
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
#include <zephyr/usb/usbd.h>
#include "usbd_init.h"
#endif

#include "probe_config.h"
#include "led.h"
#ifdef CONFIG_DEBUGPROBE_DAP
#include "probe.h"
#include "dap.h"
#endif
#ifdef CONFIG_DEBUGPROBE_CDC_UART
#include "cdc_uart.h"
#endif
#include "get_serial.h"

LOG_MODULE_REGISTER(debugprobe, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
/* Thread stacks - only UART thread needed, DAP uses usbd_dap.c's thread */
#ifdef CONFIG_DEBUGPROBE_CDC_UART
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
static struct k_thread uart_thread_data;
static k_tid_t uart_tid;

/* Ring buffers for USB-UART bridge */
RING_BUF_DECLARE(uart_tx_ringbuf, USB_TX_BUF_SIZE);
RING_BUF_DECLARE(uart_rx_ringbuf, USB_RX_BUF_SIZE);
#endif

/* USB enabled flag */
static volatile bool usb_configured = false;
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

/* GPIO device - defined in probe.c, declared extern in DAP_config.h */

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
#ifdef CONFIG_DEBUGPROBE_CDC_UART
/*
 * UART Thread - Handles CDC-UART bridge
 * Replaces: cdc_thread() with xTaskCreate(cdc_thread, "UART", ...)
 */
static void uart_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("UART thread started");

    cdc_uart_init();

    while (1) {
        /* Handle UART <-> USB CDC bridge */
        cdc_uart_task();

        /* Sleep to yield CPU - use short sleep for responsiveness */
        k_sleep(K_USEC(100));
    }
}
#endif /* CONFIG_DEBUGPROBE_CDC_UART */
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

#ifdef CONFIG_DEBUGPROBE_DAP
/*
 * GPIO Initialization
 *
 * Note: gpio_dev is initialized by probe_gpio_init() in probe.c.
 * It may be NULL on boards without gpio0 (e.g., RA4M2 uses ioport*).
 */
static int gpio_init_pins(void)
{
    int ret;

    /* Configure SWD pins first - this also initializes gpio_dev */
    ret = probe_gpio_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize probe GPIO");
        return ret;
    }

    /* LED initialization is now handled by led_init() */

    return 0;
}
#endif /* CONFIG_DEBUGPROBE_DAP */

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
/*
 * Suspend worker threads
 * Called when USB is disconnected/suspended to reduce power consumption
 */
static void suspend_threads(void)
{
#ifdef CONFIG_DEBUGPROBE_CDC_UART
    if (uart_tid) {
        k_thread_suspend(uart_tid);
        LOG_DBG("UART thread suspended");
    }
#endif
    /* Note: DAP thread is managed by usbd_dap.c via semaphores */
}

/*
 * Resume worker threads
 * Called when USB is connected/resumed
 */
static void resume_threads(void)
{
#ifdef CONFIG_DEBUGPROBE_CDC_UART
    if (uart_tid) {
        k_thread_resume(uart_tid);
        LOG_DBG("UART thread resumed");
    }
#endif
    /* Note: DAP thread is managed by usbd_dap.c via semaphores */
}

/* USB device context */
static struct usbd_context *usbd_ctx;

/*
 * USB Message Callback
 * Handles USB connection state changes using new stack message system
 */
static void usb_msg_cb(struct usbd_context *const ctx,
                       const struct usbd_msg *msg)
{
    static bool was_configured = false;

    LOG_DBG("USB message: %s", usbd_msg_type_string(msg->type));

    switch (msg->type) {
    case USBD_MSG_CONFIGURATION:
        LOG_INF("USB configured");
        usb_configured = true;
        led_usb_connected(true);
        if (!was_configured) {
            resume_threads();
        }
        was_configured = true;
        break;

    case USBD_MSG_RESET:
        LOG_INF("USB reset");
        usb_configured = false;
        led_usb_connected(false);
        /* Keep threads running during reset - they'll be needed once reconfigured */
        break;

    case USBD_MSG_SUSPEND:
        LOG_INF("USB suspended");
        led_usb_connected(false);
        if (was_configured) {
            suspend_threads();
        }
        break;

    case USBD_MSG_RESUME:
        LOG_INF("USB resumed");
        led_usb_connected(true);
        if (was_configured) {
            resume_threads();
        }
        break;

    case USBD_MSG_CDC_ACM_LINE_CODING:
        /* Baud rate change - handled by cdc_uart via uart_line_ctrl_get */
        LOG_DBG("CDC ACM line coding changed");
        break;

    case USBD_MSG_CDC_ACM_CONTROL_LINE_STATE:
        /* DTR/RTS change */
        LOG_DBG("CDC ACM control line state changed");
        break;

    default:
        break;
    }
}
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

/*
 * Main Entry Point
 * Replaces FreeRTOS main() with Zephyr threading
 */
int main(void)
{
    char serial[17];
    int ret;

    /* Initialize LEDs first for visual feedback */
    ret = led_init();
    if (ret < 0) {
        LOG_WRN("LED init failed: %d", ret);
    }

    LOG_INF("Debug Probe (Zephyr) starting...");
    LOG_INF("Version: %s", DEBUGPROBE_VERSION);

    /* Get unique serial number */
    get_serial_string(serial, sizeof(serial));
    LOG_INF("Serial: %s", serial);

#ifdef CONFIG_USB_DEVICE_STACK_NEXT

#ifdef CONFIG_DEBUGPROBE_DAP
    /* Initialize GPIO pins */
    ret = gpio_init_pins();
    if (ret < 0) {
        LOG_ERR("GPIO init failed: %d", ret);
        return ret;
    }

    /* Initialize DAP */
    dap_init();
    LOG_INF("DAP initialized");
#endif

    /* Initialize USB device with new stack */
    usbd_ctx = debugprobe_usbd_init(usb_msg_cb);
    if (usbd_ctx == NULL) {
        LOG_ERR("Failed to initialize USB device");
        return -ENODEV;
    }

    /* Enable USB device */
    ret = usbd_enable(usbd_ctx);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return ret;
    }
    LOG_INF("USB enabled");

    /* Wait for USB to be configured before printing welcome message */
    {
        const struct device *cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
        int timeout = 30; /* 3 seconds max */

        while (!usb_configured && timeout > 0) {
            k_sleep(K_MSEC(100));
            timeout--;
        }

        /* Wait a bit more for terminal to be ready, then print welcome */
        if (usb_configured) {
            uint32_t dtr = 0;
            timeout = 50; /* 5 seconds for DTR - give user time to connect */

            while (timeout > 0) {
                if (device_is_ready(cdc_dev)) {
                    uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);
                    if (dtr) {
                        break;
                    }
                }
                k_sleep(K_MSEC(100));
                timeout--;
            }

            /* Print welcome message with console test */
            printk("\n");
            printk("    DEBUGPROBE CONSOLE TEST\n");
            printk("Version: %s (Zephyr)\n", DEBUGPROBE_VERSION);
            printk("Serial: %s\n", serial);
            printk("\n");
            printk("If you see this message, the console is working!\n");
            printk("Waiting 3 seconds before continuing, Flash LEDs...\n");

            /* Flash LEDs to show we're in console test mode */
            for (int i = 0; i < 6; i++) {
                led_dap_running(i % 2 == 0);
                k_sleep(K_MSEC(500));
            }

            printk("Console test complete. Starting DAP...\n\n");
        }
    }

    /*
     * Create threads
     *
     * Note: USB handling is event-driven via Zephyr callbacks (no thread needed).
     * DAP thread is created by usbd_dap.c when the USB class is initialized.
     * Only UART thread is created here for CDC-UART bridge.
     */

#ifdef CONFIG_DEBUGPROBE_CDC_UART
    /* UART thread starts suspended - resumed when USB is configured */
    uart_tid = k_thread_create(&uart_thread_data, uart_thread_stack,
                               K_THREAD_STACK_SIZEOF(uart_thread_stack),
                               uart_thread_entry, NULL, NULL, NULL,
                               UART_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(uart_tid, "uart_thread");
    k_thread_suspend(uart_tid);
    LOG_INF("UART thread created");
#endif

    /* Resume threads if USB was already configured during init */
    if (usb_configured) {
        LOG_INF("USB already configured, resuming threads");
        resume_threads();
    }
#else
    LOG_INF("Running in simulation mode (no USB)");
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

    /* Main loop - can be used for monitoring or additional tasks */
    while (1) {
        k_sleep(K_SECONDS(1));
        /* Could add watchdog feed here */
    }

    return 0;
}

#ifdef CONFIG_USB_DEVICE_STACK_NEXT
/*
 * USB Configured Check
 */
bool is_usb_configured(void)
{
    return usb_configured;
}
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */
