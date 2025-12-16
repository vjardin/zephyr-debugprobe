/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - Vendor-specific DAP Commands
 *
 * Implements vendor-specific extensions to the CMSIS-DAP protocol.
 * These commands mirror the original debugprobe vendor extensions.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "probe_config.h"
#include "probe.h"
#include "dap.h"

LOG_MODULE_REGISTER(dap_vendor, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Vendor command IDs and PROBE_VENDOR_* definitions are in DAP_config.h
 * (included via probe.h).
 */

/* LED IDs */
#define LED_ID_DAP              0
#define LED_ID_UART_TX          1
#define LED_ID_UART_RX          2
#define LED_ID_CONNECTED        3
#define LED_ID_RUNNING          4

/* GPIO device reference - defined in probe.c, declared extern in DAP_config.h */

/*
 * Initialize vendor command subsystem
 */
static void vendor_init(void)
{
    static bool initialized = false;

    if (!initialized) {
        gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        if (!device_is_ready(gpio_dev)) {
            gpio_dev = NULL;
        }
        initialized = true;
    }
}

/*
 * Vendor: Get Version (PROBE_VENDOR_VERSION)
 * Returns firmware version string
 */
static uint32_t dap_vendor_version(const uint8_t *request, uint8_t *response)
{
    const char *version = DEBUGPROBE_VERSION;
    uint32_t len = strlen(version);

    ARG_UNUSED(request);

    response[0] = PROBE_VENDOR_VERSION;
    response[1] = DAP_OK;
    response[2] = (uint8_t)len;
    memcpy(&response[3], version, len);

    return 3 + len;
}

/*
 * Vendor: Reset Configuration (PROBE_VENDOR_RESET_CONFIG)
 * Reset all DAP settings to defaults
 */
static uint32_t dap_vendor_reset_config(const uint8_t *request, uint8_t *response)
{
    ARG_UNUSED(request);

    response[0] = PROBE_VENDOR_RESET_CONFIG;
    response[1] = DAP_OK;

    /* Reset probe to default state */
    probe_set_swj_clock(DAP_DEFAULT_SWJ_CLOCK);
    probe_pins_hiz();

    LOG_INF("DAP configuration reset to defaults");

    return 2;
}

/*
 * Vendor: LED Control (PROBE_VENDOR_LED_CTRL)
 * Manual LED control for testing/indication
 *
 * Request: [cmd] [led_id] [state]
 *   led_id: 0=DAP, 1=TX, 2=RX, 3=Connected, 4=Running
 *   state: 0=off, 1=on
 */
static uint32_t dap_vendor_led_ctrl(const uint8_t *request, uint8_t *response)
{
    uint8_t led_id = request[1];
    uint8_t led_state = request[2];
    int pin = -1;

    response[0] = PROBE_VENDOR_LED_CTRL;

    vendor_init();

    switch (led_id) {
    case LED_ID_DAP:
#ifdef PROBE_DAP_LED
        pin = PROBE_DAP_LED;
#endif
        break;
    case LED_ID_UART_TX:
#ifdef PROBE_UART_TX_LED
        pin = PROBE_UART_TX_LED;
#endif
        break;
    case LED_ID_UART_RX:
#ifdef PROBE_UART_RX_LED
        pin = PROBE_UART_RX_LED;
#endif
        break;
    case LED_ID_CONNECTED:
#ifdef PROBE_DAP_CONNECTED_LED
        pin = PROBE_DAP_CONNECTED_LED;
#endif
        break;
    case LED_ID_RUNNING:
#ifdef PROBE_DAP_RUNNING_LED
        pin = PROBE_DAP_RUNNING_LED;
#endif
        break;
    default:
        response[1] = DAP_ERROR;
        return 2;
    }

    if (pin >= 0 && gpio_dev != NULL) {
        gpio_pin_set(gpio_dev, pin, led_state ? 1 : 0);
        response[1] = DAP_OK;
    } else {
        response[1] = DAP_ERROR;
    }

    LOG_DBG("LED control: id=%d state=%d pin=%d", led_id, led_state, pin);

    return 2;
}

/*
 * Vendor: Target Voltage (PROBE_VENDOR_TARGET_VOLTAGE)
 * Read target voltage (if ADC available)
 *
 * Response: [cmd] [status] [voltage_mv_lo] [voltage_mv_hi]
 */
static uint32_t dap_vendor_target_voltage(const uint8_t *request, uint8_t *response)
{
    uint16_t voltage_mv = 0;

    ARG_UNUSED(request);

    response[0] = PROBE_VENDOR_TARGET_VOLTAGE;

    /* Target voltage reading would require ADC
     * For now, return 0 (not measured) or 3300 (assumed 3.3V) */
#ifdef CONFIG_DEBUGPROBE_TARGET_VOLTAGE_ADC
    /* ADC reading would go here */
    voltage_mv = 3300;
    response[1] = DAP_OK;
#else
    /* No ADC, return 0 to indicate not measured */
    voltage_mv = 0;
    response[1] = DAP_OK;
#endif

    response[2] = (uint8_t)(voltage_mv & 0xFF);
    response[3] = (uint8_t)(voltage_mv >> 8);

    return 4;
}

/*
 * Vendor: Reset Target (PROBE_VENDOR_RESET_TARGET)
 * Perform target reset with configurable timing
 *
 * Request: [cmd] [reset_type] [duration_lo] [duration_hi]
 *   reset_type: 0=pulse, 1=hold, 2=release
 *   duration: reset pulse duration in milliseconds (for pulse type)
 */
static uint32_t dap_vendor_reset_target(const uint8_t *request, uint8_t *response)
{
    uint8_t reset_type = request[1];
    uint16_t duration = (uint16_t)request[2] | ((uint16_t)request[3] << 8);

    response[0] = PROBE_VENDOR_RESET_TARGET;
    response[1] = DAP_OK;

    switch (reset_type) {
    case 0: /* Pulse */
        probe_set_reset(true);
        if (duration > 0) {
            k_msleep(duration);
        } else {
            k_busy_wait(10000);  /* 10ms default */
        }
        probe_set_reset(false);
        LOG_INF("Target reset pulse: %u ms", duration ? duration : 10);
        break;
    case 1: /* Hold */
        probe_set_reset(true);
        LOG_INF("Target reset held");
        break;
    case 2: /* Release */
        probe_set_reset(false);
        LOG_INF("Target reset released");
        break;
    default:
        response[1] = DAP_ERROR;
        break;
    }

    return 2;
}

/*
 * Vendor: Boot Mode (PROBE_VENDOR_BOOT_MODE)
 * Configure target boot mode pins (if available)
 *
 * Request: [cmd] [boot_mode]
 */
static uint32_t dap_vendor_boot_mode(const uint8_t *request, uint8_t *response)
{
    uint8_t boot_mode = request[1];

    response[0] = PROBE_VENDOR_BOOT_MODE;

    /* Boot mode control requires hardware-specific pins */
    /* This is a placeholder for implementations that support it */
    ARG_UNUSED(boot_mode);

    response[1] = DAP_OK;
    LOG_DBG("Boot mode set: %d (not implemented)", boot_mode);

    return 2;
}

/*
 * Vendor: Get Status (PROBE_VENDOR_GET_STATUS)
 * Return probe status information
 *
 * Response: [cmd] [status] [flags] [clock_lo] [clock_mid_lo] [clock_mid_hi] [clock_hi]
 */
static uint32_t dap_vendor_get_status(const uint8_t *request, uint8_t *response)
{
    uint32_t clock;
    uint8_t flags = 0;

    ARG_UNUSED(request);

    response[0] = PROBE_VENDOR_GET_STATUS;
    response[1] = DAP_OK;

    /* Build status flags */
    if (probe_is_connected()) {
        flags |= 0x01;  /* Target connected */
    }

    response[2] = flags;

    /* Current SWJ clock */
    clock = probe_get_swj_clock();
    response[3] = (uint8_t)(clock >> 0);
    response[4] = (uint8_t)(clock >> 8);
    response[5] = (uint8_t)(clock >> 16);
    response[6] = (uint8_t)(clock >> 24);

    return 7;
}

/*
 * Vendor: Set System Clock (PROBE_VENDOR_SET_SYSCLK)
 * Configure probe system clock (for PIO timing, etc.)
 *
 * Request: [cmd] [clock_lo] [clock_mid_lo] [clock_mid_hi] [clock_hi]
 */
static uint32_t dap_vendor_set_sysclk(const uint8_t *request, uint8_t *response)
{
    uint32_t clock = (uint32_t)request[1] |
                     ((uint32_t)request[2] << 8) |
                     ((uint32_t)request[3] << 16) |
                     ((uint32_t)request[4] << 24);

    response[0] = PROBE_VENDOR_SET_SYSCLK;

    /* System clock configuration is hardware-specific */
    /* This is primarily useful for PIO timing adjustments */
    ARG_UNUSED(clock);

    response[1] = DAP_OK;
    LOG_DBG("System clock request: %u Hz (not implemented)", clock);

    return 2;
}

/*
 * Process vendor-specific DAP command
 */
uint32_t dap_vendor_command(const uint8_t *request, uint8_t *response)
{
    uint8_t cmd = request[0];

    switch (cmd) {
    case PROBE_VENDOR_VERSION:
        return dap_vendor_version(request, response);
    case PROBE_VENDOR_RESET_CONFIG:
        return dap_vendor_reset_config(request, response);
    case PROBE_VENDOR_LED_CTRL:
        return dap_vendor_led_ctrl(request, response);
    case PROBE_VENDOR_TARGET_VOLTAGE:
        return dap_vendor_target_voltage(request, response);
    case PROBE_VENDOR_RESET_TARGET:
        return dap_vendor_reset_target(request, response);
    case PROBE_VENDOR_BOOT_MODE:
        return dap_vendor_boot_mode(request, response);
    case PROBE_VENDOR_GET_STATUS:
        return dap_vendor_get_status(request, response);
    case PROBE_VENDOR_SET_SYSCLK:
        return dap_vendor_set_sysclk(request, response);
    default:
        /* Unknown vendor command */
        response[0] = cmd;
        response[1] = DAP_ERROR;
        LOG_WRN("Unknown vendor command: 0x%02X", cmd);
        return 2;
    }
}
