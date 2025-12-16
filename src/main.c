/* SPDX-License-Identifier: MIT
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
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include "probe_config.h"
#include "probe.h"
#include "cdc_uart.h"
#include "dap.h"
#include "get_serial.h"
#include "usbd_init.h"

LOG_MODULE_REGISTER(debugprobe, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread stacks */
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(dap_thread_stack, DAP_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);

/* Thread control blocks */
static struct k_thread uart_thread_data;
static struct k_thread dap_thread_data;
static struct k_thread usb_thread_data;

/* Thread IDs */
static k_tid_t uart_tid;
static k_tid_t dap_tid;
static k_tid_t usb_tid;

/* Synchronization primitives */
K_SEM_DEFINE(dap_request_sem, 0, 1);
K_SEM_DEFINE(dap_response_sem, 0, 1);

/* DAP buffers */
static uint8_t tx_data_buffer[CFG_TUD_HID_EP_BUFSIZE];
static uint8_t rx_data_buffer[CFG_TUD_HID_EP_BUFSIZE];

/* Ring buffers for USB-UART bridge */
RING_BUF_DECLARE(uart_tx_ringbuf, USB_TX_BUF_SIZE);
RING_BUF_DECLARE(uart_rx_ringbuf, USB_RX_BUF_SIZE);

/* GPIO device - defined in probe.c, declared extern in DAP_config.h */

/* USB enabled flag */
static volatile bool usb_configured = false;

/*
 * LED Control
 */
#ifdef CONFIG_DEBUGPROBE_DAP_LED
static void dap_led_set(bool on)
{
    if (gpio_dev && PROBE_DAP_LED >= 0) {
        gpio_pin_set(gpio_dev, PROBE_DAP_LED, on ? 1 : 0);
    }
}
#else
#define dap_led_set(x) do {} while(0)
#endif

/*
 * USB Thread - Handles USB device tasks
 * Replaces: usb_thread() with xTaskCreate(usb_thread, "TUD", ...)
 */
static void usb_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("USB thread started");

    while (1) {
        /* USB stack handling is done by Zephyr's workqueue
         * We just need to yield and let the USB subsystem work */
        k_sleep(K_MSEC(1));
        
        /* Check USB configuration status */
        /* This is handled by Zephyr's USB stack callbacks */
    }
}

/*
 * DAP Thread - Processes DAP requests
 * Replaces: dap_thread() with xTaskCreate(dap_thread, "DAP", ...)
 */
static void dap_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("DAP thread started");

    while (1) {
        /* Wait for DAP request */
        if (k_sem_take(&dap_request_sem, K_FOREVER) == 0) {
            uint32_t response_len;

            dap_led_set(true);
            
            /* Process DAP command */
            response_len = dap_process_request(rx_data_buffer, 
                                               sizeof(rx_data_buffer),
                                               tx_data_buffer,
                                               sizeof(tx_data_buffer));
            
            /* Signal response ready */
            k_sem_give(&dap_response_sem);
            
            dap_led_set(false);
        }
    }
}

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

/*
 * GPIO Initialization
 */
static int gpio_init_pins(void)
{
    int ret;

    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

#ifdef CONFIG_DEBUGPROBE_DAP_LED
    if (PROBE_DAP_LED >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_DAP_LED, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure DAP LED pin");
        }
    }
#endif

#ifdef CONFIG_DEBUGPROBE_TX_LED
    if (PROBE_UART_TX_LED >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_UART_TX_LED, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure TX LED pin");
        }
    }
#endif

#ifdef CONFIG_DEBUGPROBE_RX_LED
    if (PROBE_UART_RX_LED >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_UART_RX_LED, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure RX LED pin");
        }
    }
#endif

    /* Configure SWD pins */
    ret = probe_gpio_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize probe GPIO");
        return ret;
    }

    return 0;
}

/*
 * Suspend worker threads
 * Called when USB is disconnected/suspended to reduce power consumption
 */
static void suspend_threads(void)
{
    if (uart_tid) {
        k_thread_suspend(uart_tid);
        LOG_DBG("UART thread suspended");
    }
    if (dap_tid) {
        k_thread_suspend(dap_tid);
        LOG_DBG("DAP thread suspended");
    }
}

/*
 * Resume worker threads
 * Called when USB is connected/resumed
 */
static void resume_threads(void)
{
    if (uart_tid) {
        k_thread_resume(uart_tid);
        LOG_DBG("UART thread resumed");
    }
    if (dap_tid) {
        k_thread_resume(dap_tid);
        LOG_DBG("DAP thread resumed");
    }
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
        if (!was_configured) {
            resume_threads();
        }
        was_configured = true;
        break;

    case USBD_MSG_RESET:
        LOG_INF("USB reset");
        usb_configured = false;
        /* Keep threads running during reset - they'll be needed once reconfigured */
        break;

    case USBD_MSG_SUSPEND:
        LOG_INF("USB suspended");
        if (was_configured) {
            suspend_threads();
        }
        break;

    case USBD_MSG_RESUME:
        LOG_INF("USB resumed");
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

/*
 * Main Entry Point
 * Replaces FreeRTOS main() with Zephyr threading
 */
int main(void)
{
    int ret;
    char serial[17];

    LOG_INF("Debug Probe (Zephyr) starting...");
    LOG_INF("Version: %s", DEBUGPROBE_VERSION);

    /* Get unique serial number */
    get_serial_string(serial, sizeof(serial));
    LOG_INF("Serial: %s", serial);

    /* Initialize GPIO pins */
    ret = gpio_init_pins();
    if (ret < 0) {
        LOG_ERR("GPIO init failed: %d", ret);
        return ret;
    }

    /* Initialize DAP */
    dap_init();
    LOG_INF("DAP initialized");

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

    /* Wait for USB to be ready */
    k_sleep(K_MSEC(100));

    /*
     * Create threads
     * 
     * In FreeRTOS this was:
     *   xTaskCreate(usb_thread, "TUD", configMINIMAL_STACK_SIZE, NULL, TUD_TASK_PRIO, &tud_taskhandle);
     *   xTaskCreate(cdc_thread, "UART", configMINIMAL_STACK_SIZE, NULL, UART_TASK_PRIO, &uart_taskhandle);
     *   xTaskCreate(dap_thread, "DAP", configMINIMAL_STACK_SIZE, NULL, DAP_TASK_PRIO, &dap_taskhandle);
     *
     * In Zephyr we use k_thread_create():
     */
    
    /* USB thread always runs (manages USB stack) */
    usb_tid = k_thread_create(&usb_thread_data, usb_thread_stack,
                              K_THREAD_STACK_SIZEOF(usb_thread_stack),
                              usb_thread_entry, NULL, NULL, NULL,
                              USB_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(usb_tid, "usb_thread");

    /* UART and DAP threads start suspended - resumed when USB is configured */
    uart_tid = k_thread_create(&uart_thread_data, uart_thread_stack,
                               K_THREAD_STACK_SIZEOF(uart_thread_stack),
                               uart_thread_entry, NULL, NULL, NULL,
                               UART_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(uart_tid, "uart_thread");
    k_thread_suspend(uart_tid);

    dap_tid = k_thread_create(&dap_thread_data, dap_thread_stack,
                              K_THREAD_STACK_SIZEOF(dap_thread_stack),
                              dap_thread_entry, NULL, NULL, NULL,
                              DAP_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(dap_tid, "dap_thread");
    k_thread_suspend(dap_tid);

    LOG_INF("All threads created (UART/DAP suspended until USB configured)");

    /* Main loop - can be used for monitoring or additional tasks */
    while (1) {
        k_sleep(K_SECONDS(1));
        /* Could add watchdog feed here */
    }

    return 0;
}

/*
 * DAP Request Handler
 * Called from USB HID/Bulk endpoint handler
 */
void dap_handle_request(const uint8_t *request, uint32_t request_len,
                        uint8_t *response, uint32_t *response_len)
{
    if (request_len > sizeof(rx_data_buffer)) {
        request_len = sizeof(rx_data_buffer);
    }

    memcpy(rx_data_buffer, request, request_len);
    
    /* Signal DAP thread */
    k_sem_give(&dap_request_sem);
    
    /* Wait for response */
    if (k_sem_take(&dap_response_sem, K_MSEC(1000)) == 0) {
        memcpy(response, tx_data_buffer, DAP_PACKET_SIZE);
        *response_len = DAP_PACKET_SIZE;
    } else {
        /* Timeout - return error response */
        response[0] = 0xFF;
        *response_len = 1;
    }
}

/*
 * USB Configured Check
 */
bool is_usb_configured(void)
{
    return usb_configured;
}
