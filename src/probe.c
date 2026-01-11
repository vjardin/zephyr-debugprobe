/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - SWD Probe Implementation
 *
 * Port of probe.c from pico-sdk/FreeRTOS to Zephyr RTOS
 * Original: https://github.com/raspberrypi/debugprobe/blob/master/src/probe.c
 *
 * This implements the low-level SWD (Serial Wire Debug) interface.
 * The original used RP2040's PIO for fast bit-banging.
 * This Zephyr port provides both GPIO bit-bang and PIO options.
 *
 * GPIO pins are configured via devicetree using the "debugprobe-swd" binding,
 * allowing support for multiple GPIO controllers (e.g., RA4M2 with ioport0-5).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "probe_config.h"
#include "probe.h"

#ifdef CONFIG_SOC_RP2040
#include "pio_swd.h"
#include <hardware/gpio.h>
#endif

LOG_MODULE_REGISTER(probe, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Devicetree-based GPIO configuration
 *
 * This allows pins to be on different GPIO controllers, which is required
 * for boards like RA4M2 where SWD pins span multiple ports.
 */
#define SWD_NODE DT_NODELABEL(debugprobe_swd)

#if DT_NODE_EXISTS(SWD_NODE)
/* GPIO specs from devicetree - supports multiple controllers */
static const struct gpio_dt_spec swclk_gpio = GPIO_DT_SPEC_GET(SWD_NODE, swclk_gpios);
static const struct gpio_dt_spec swdio_gpio = GPIO_DT_SPEC_GET(SWD_NODE, swdio_gpios);

/* Optional pins - use _GET_OR with empty spec for optional properties */
#if DT_NODE_HAS_PROP(SWD_NODE, reset_gpios)
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET(SWD_NODE, reset_gpios);
#define HAS_RESET_GPIO 1
#else
#define HAS_RESET_GPIO 0
#endif

#if DT_NODE_HAS_PROP(SWD_NODE, swdir_gpios)
static const struct gpio_dt_spec swdir_gpio = GPIO_DT_SPEC_GET(SWD_NODE, swdir_gpios);
#define HAS_SWDIR_GPIO 1
#else
#define HAS_SWDIR_GPIO 0
#endif

#if DT_NODE_HAS_PROP(SWD_NODE, io_enable_gpios)
static const struct gpio_dt_spec io_en_gpio = GPIO_DT_SPEC_GET(SWD_NODE, io_enable_gpios);
#define HAS_IO_EN_GPIO 1
#else
#define HAS_IO_EN_GPIO 0
#endif

#define USE_DT_GPIO 1
#else
/* Fallback: no devicetree node - use legacy gpio0 approach */
#define USE_DT_GPIO 0
#endif

/*
 * GPIO device for LED control and legacy SWD fallback.
 * This is always defined because LEDs (PROBE_DAP_LED, etc.)
 * are controlled via gpio0 by main.c, dap_vendor.c, etc.
 * The extern declaration is in DAP_config.h.
 */
const struct device *gpio_dev;

/* PIO acceleration mode (RP2040 only) */
#ifdef CONFIG_SOC_RP2040
static bool use_pio_swd = false;
#endif

/* Current SWD clock frequency */
static uint32_t swj_clock = DAP_DEFAULT_SWJ_CLOCK;

/* Delay between clock edges (nanoseconds) */
static uint32_t clock_delay_ns;

/* SWD state */
static bool swd_connected = false;

/*
 * Calculate clock delay based on frequency
 */
static void update_clock_delay(void)
{
    if (swj_clock > 0) {
        /* Half period in nanoseconds */
        clock_delay_ns = 500000000UL / swj_clock;
    } else {
        clock_delay_ns = 500; /* Default 1MHz */
    }
}

/*
 * Inline delay for clock timing
 * Note: This is a simple busy-wait. For RP2040, PIO is much better.
 */
static inline void clock_delay(void)
{
    /* For very fast clocks, this may not be accurate */
    if (clock_delay_ns > 100) {
        k_busy_wait(clock_delay_ns / 1000);
    }
}

/*
 * Set SWDIO direction - Debug Probe Hardware Model
 *
 * The Debug Probe has SEPARATE input/output paths with auto-direction level shifter:
 *   - SWCLK (GPIO12): Clock output
 *   - SWDI (GPIO13): Input from target (always reads from target side)
 *   - SWDIO (GPIO14): Output to target
 *
 * The level shifter auto-senses direction - no explicit OE/direction control needed.
 * SWDI is always receiving from the target, SWDIO is always transmitting to target.
 * Direction control is just about enabling/disabling our output driver (pindir).
 *
 * To write: Set SWDIO pindir=output, write to SWDIO
 * To read: Set SWDIO pindir=input (tristate), read from SWDI
 */
static inline void swdio_set_dir_out(void)
{
#ifdef CONFIG_SOC_RP2040
    /* When PIO is enabled, direction is controlled by PIO via `out pindirs, 1`.
     * Do NOT call gpio_set_function here - it would override the PIO function!
     * Only set direction for GPIO bit-bang mode.
     */
    if (!use_pio_swd) {
        gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(PROBE_SWDIO_PIN, GPIO_OUT);
    }
#elif USE_DT_GPIO
    gpio_pin_configure_dt(&swdio_gpio, GPIO_OUTPUT);
#else
    gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_OUTPUT);
#endif
}

static inline void swdio_set_dir_in(void)
{
#ifdef CONFIG_SOC_RP2040
    /* When PIO is enabled, direction is controlled by PIO.
     * Only set direction for GPIO bit-bang mode.
     */
    if (!use_pio_swd) {
        gpio_set_dir(PROBE_SWDIO_PIN, GPIO_IN);
    }
#elif USE_DT_GPIO
    gpio_pin_configure_dt(&swdio_gpio, GPIO_INPUT);
#else
    gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_INPUT);
#endif
}

/*
 * SWCLK control - using devicetree GPIO specs
 */
static inline void swclk_set(int val)
{
#ifdef CONFIG_SOC_RP2040
    gpio_put(PROBE_SWCLK_PIN, val);
#elif USE_DT_GPIO
    gpio_pin_set_dt(&swclk_gpio, val);
#else
    gpio_pin_set(gpio_dev, PROBE_SWCLK_PIN, val);
#endif
}

/*
 * SWDIO control - using devicetree GPIO specs
 */
static inline void swdio_set(int val)
{
#ifdef CONFIG_SOC_RP2040
    gpio_put(PROBE_SWDIO_PIN, val);
#elif USE_DT_GPIO
    gpio_pin_set_dt(&swdio_gpio, val);
#else
    gpio_pin_set(gpio_dev, PROBE_SWDIO_PIN, val);
#endif
}

static inline int swdio_get(void)
{
#ifdef CONFIG_SOC_RP2040
    /* Read from SWDI (GPIO13), the separate input path on Debug Probe */
#if defined(PROBE_SWDI_PIN) && (PROBE_SWDI_PIN >= 0)
    return gpio_get(PROBE_SWDI_PIN);
#else
    return gpio_get(PROBE_SWDIO_PIN);
#endif
#elif USE_DT_GPIO
    return gpio_pin_get_dt(&swdio_gpio);
#else
    return gpio_pin_get(gpio_dev, PROBE_SWDIO_PIN);
#endif
}

/*
 * TDI control (JTAG data input to target)
 */
static inline void tdi_set(int val)
{
#if PROBE_JTAG_TDI_AVAILABLE && !USE_DT_GPIO
    gpio_pin_set(gpio_dev, PROBE_TDI_PIN, val);
#else
    ARG_UNUSED(val);
#endif
}

/*
 * TDO read (JTAG data output from target)
 */
static inline int tdo_get(void)
{
#if PROBE_JTAG_TDO_AVAILABLE && !USE_DT_GPIO
    return gpio_pin_get(gpio_dev, PROBE_TDO_PIN);
#else
    return 0;
#endif
}

/*
 * Write a single bit on SWD
 */
static inline void swd_write_bit(int bit)
{
    swdio_set(bit);
    clock_delay();
    swclk_set(1);
    clock_delay();
    swclk_set(0);
}

/*
 * Read a single bit from SWD
 */
static inline int swd_read_bit(void)
{
    int bit;
    clock_delay();
    swclk_set(1);
    bit = swdio_get();
    clock_delay();
    swclk_set(0);
    return bit;
}

/*
 * Initialize probe GPIO using devicetree configuration
 */
int probe_gpio_init(void)
{
    int ret;

    /*
     * Initialize gpio_dev for LED control (always uses gpio0).
     * This is needed by main.c, dap_vendor.c, etc. even when
     * SWD pins use devicetree-based multi-controller support.
     */
#if DT_NODE_EXISTS(DT_NODELABEL(gpio0))
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_WRN("GPIO0 device not ready - LED control disabled");
        gpio_dev = NULL;
    }
#else
    gpio_dev = NULL;
#endif

#if USE_DT_GPIO
    /* Verify SWCLK GPIO is ready */
    if (!gpio_is_ready_dt(&swclk_gpio)) {
        LOG_ERR("SWCLK GPIO device not ready");
        return -ENODEV;
    }

    /* Verify SWDIO GPIO is ready */
    if (!gpio_is_ready_dt(&swdio_gpio)) {
        LOG_ERR("SWDIO GPIO device not ready");
        return -ENODEV;
    }

    /* Configure SWCLK as output, initially low */
    ret = gpio_pin_configure_dt(&swclk_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure SWCLK pin: %d", ret);
        return ret;
    }
    LOG_DBG("SWCLK configured on %s pin %d",
            swclk_gpio.port->name, swclk_gpio.pin);

    /* Configure SWDIO as output initially */
    ret = gpio_pin_configure_dt(&swdio_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure SWDIO pin: %d", ret);
        return ret;
    }
    LOG_DBG("SWDIO configured on %s pin %d",
            swdio_gpio.port->name, swdio_gpio.pin);

#if HAS_RESET_GPIO
    /* Configure reset pin if available */
    if (gpio_is_ready_dt(&reset_gpio)) {
        ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
        if (ret < 0) {
            LOG_WRN("Failed to configure RESET pin: %d", ret);
        } else {
            LOG_DBG("RESET configured on %s pin %d",
                    reset_gpio.port->name, reset_gpio.pin);
        }
    }
#endif

#if HAS_SWDIR_GPIO
    /* Configure direction control pin if available
     * SWDIR is active low for output: start with SWDIR=0 (output mode)
     */
    if (gpio_is_ready_dt(&swdir_gpio)) {
        ret = gpio_pin_configure_dt(&swdir_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure SWDIR pin: %d", ret);
        } else {
            LOG_DBG("SWDIR configured on %s pin %d (active low)",
                    swdir_gpio.port->name, swdir_gpio.pin);
        }
    }
#endif

#if HAS_IO_EN_GPIO
    /* Configure I/O enable pin if available */
    if (gpio_is_ready_dt(&io_en_gpio)) {
        ret = gpio_pin_configure_dt(&io_en_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure IO_EN pin: %d", ret);
        } else {
            LOG_DBG("IO_EN configured on %s pin %d",
                    io_en_gpio.port->name, io_en_gpio.pin);
        }
    }
#endif

#else /* Legacy non-DT path */
    /* gpio_dev already initialized above for LED control */
    if (!gpio_dev) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    /* Configure SWCLK as output, initially low */
    ret = gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure SWCLK pin");
        return ret;
    }

    /* Configure SWDIO as output initially */
    ret = gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure SWDIO pin");
        return ret;
    }

#if defined(PROBE_SWDI_PIN) && (PROBE_SWDI_PIN >= 0)
    /* Configure SWDI as input for reading from target
     * Debug Probe has separate input/output paths through level shifter
     */
    gpio_init(PROBE_SWDI_PIN);
    gpio_set_dir(PROBE_SWDI_PIN, GPIO_IN);
    gpio_pull_up(PROBE_SWDI_PIN);
    LOG_DBG("SWDI configured on GPIO%d as input", PROBE_SWDI_PIN);
#endif

#ifdef PROBE_SWDIR_PIN
    /* Configure direction control pin if present
     * SWDIR is active low for output: start with SWDIR=0 (output mode)
     */
    if (PROBE_SWDIR_PIN >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_SWDIR_PIN, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure SWDIR pin");
        }
    }
#endif

#ifdef PROBE_RESET_PIN
    /* Configure reset pin */
    if (PROBE_RESET_PIN >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_RESET_PIN,
                                 GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
        if (ret < 0) {
            LOG_WRN("Failed to configure RESET pin");
        }
    }
#endif

#ifdef PROBE_IO_OEN
    /* Configure output enable for level shifters */
    if (PROBE_IO_OEN >= 0) {
        ret = gpio_pin_configure(gpio_dev, PROBE_IO_OEN, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure IO_OEN pin");
        }
    }
#endif
#endif /* USE_DT_GPIO */

    update_clock_delay();

#ifdef CONFIG_SOC_RP2040
    /* Initialize PIO SWD for hardware acceleration.
     * PIO provides deterministic timing required for SWD protocol compliance,
     * especially for APPROTECT targets like nRF91 that need precise timing
     * for dormant wakeup sequences.
     *
     * The original debugprobe uses PIO for all SWD operations.
     */
    if (pio_swd_init() == 0) {
        use_pio_swd = true;
        LOG_INF("PIO SWD acceleration enabled");
    } else {
        use_pio_swd = false;
        LOG_WRN("PIO SWD init failed, using GPIO bit-bang (may have timing issues)");
    }
#endif

    LOG_INF("Probe GPIO initialized (devicetree: %s)",
            USE_DT_GPIO ? "yes" : "no");

    return 0;
}

/*
 * Set SWJ clock frequency
 */
void probe_set_swj_clock(uint32_t clock_hz)
{
    swj_clock = clock_hz;
    update_clock_delay();

#ifdef CONFIG_SOC_RP2040
    if (use_pio_swd) {
        pio_swd_set_freq(clock_hz);
    }
#endif

    LOG_INF("SWJ clock set to %u Hz", clock_hz);
}

/*
 * Get current SWJ clock frequency
 */
uint32_t probe_get_swj_clock(void)
{
    return swj_clock;
}

/*
 * SWJ Sequence - Output sequence on SWDIO/SWCLK
 * Used for line reset, switch sequences, etc.
 */
void probe_swj_sequence(uint32_t count, const uint8_t *data)
{
#ifdef CONFIG_SOC_RP2040
    if (use_pio_swd) {
        pio_swd_sequence(count, data);
        return;
    }
#endif

    /* GPIO bit-bang fallback */
    uint32_t val;
    uint32_t n;

    swdio_set_dir_out();

    while (count > 0) {
        val = *data++;
        n = (count > 8) ? 8 : count;
        count -= n;

        while (n--) {
            swd_write_bit(val & 1);
            val >>= 1;
        }
    }
}

/*
 * SWD Transfer
 * Perform a single SWD read or write transaction
 *
 * @param request  Request byte (contains APnDP, RnW, A[2:3], parity)
 * @param data     Pointer to data (write) or result (read)
 * @return         ACK value (OK=1, WAIT=2, FAULT=4) or error
 */
uint8_t probe_swd_transfer(uint32_t request, uint32_t *data)
{
#ifdef CONFIG_SOC_RP2040
    if (use_pio_swd) {
        return pio_swd_transfer(request, data);
    }
#endif

    /* GPIO bit-bang implementation */
    uint32_t ack;
    uint32_t bit;
    uint32_t val;
    uint32_t parity;
    uint32_t n;

    /* Start bit */
    swdio_set_dir_out();
    swd_write_bit(1);

    /* APnDP bit */
    swd_write_bit((request >> 0) & 1);

    /* RnW bit */
    swd_write_bit((request >> 1) & 1);

    /* A[2:3] bits */
    swd_write_bit((request >> 2) & 1);
    swd_write_bit((request >> 3) & 1);

    /* Parity bit */
    parity = ((request >> 0) & 1) ^ ((request >> 1) & 1) ^
             ((request >> 2) & 1) ^ ((request >> 3) & 1);
    swd_write_bit(parity);

    /* Stop bit */
    swd_write_bit(0);

    /* Park bit */
    swd_write_bit(1);

    /* Turnaround */
    swdio_set_dir_in();
    /* Allow level shifter to switch direction - Debug Probe has auto-direction
     * sensing which needs time to detect we've released the bus.
     * Without this delay, we read our own driven value instead of target's.
     */
    k_busy_wait(10);
    clock_delay();
    swclk_set(1);
    clock_delay();
    swclk_set(0);

    /* Read ACK */
    ack = 0;
    for (n = 0; n < 3; n++) {
        bit = swd_read_bit();
        ack |= (bit << n);
    }

    if (ack == DAP_TRANSFER_OK) {
        /* Read/write data phase */
        if (request & DAP_TRANSFER_RnW) {
            /* Read data */
            val = 0;
            parity = 0;
            for (n = 0; n < 32; n++) {
                bit = swd_read_bit();
                val |= (bit << n);
                parity ^= bit;
            }

            /* Read parity */
            bit = swd_read_bit();
            if (parity != bit) {
                ack = DAP_TRANSFER_ERROR;
            } else if (data) {
                *data = val;
            }

            /* Turnaround */
            clock_delay();
            swclk_set(1);
            clock_delay();
            swclk_set(0);
            swdio_set_dir_out();
        } else {
            /* Turnaround */
            clock_delay();
            swclk_set(1);
            clock_delay();
            swclk_set(0);
            swdio_set_dir_out();

            /* Write data */
            val = data ? *data : 0;
            parity = 0;
            for (n = 0; n < 32; n++) {
                swd_write_bit(val & 1);
                parity ^= (val & 1);
                val >>= 1;
            }

            /* Write parity */
            swd_write_bit(parity);
        }

        /* Idle cycles */
        swdio_set(0);
        for (n = 0; n < 8; n++) {
            clock_delay();
            swclk_set(1);
            clock_delay();
            swclk_set(0);
        }
        swdio_set(1);
    } else if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT) {
        /* Wait for data phase (dummy) */
        if (request & DAP_TRANSFER_RnW) {
            for (n = 0; n < 33; n++) {
                clock_delay();
                swclk_set(1);
                clock_delay();
                swclk_set(0);
            }
        }

        /* Turnaround */
        clock_delay();
        swclk_set(1);
        clock_delay();
        swclk_set(0);
        swdio_set_dir_out();

        if (!(request & DAP_TRANSFER_RnW)) {
            /* Write dummy data on WAIT/FAULT */
            for (n = 0; n < 33; n++) {
                swd_write_bit(0);
            }
        }

        /* Idle cycles */
        swdio_set(0);
        for (n = 0; n < 8; n++) {
            clock_delay();
            swclk_set(1);
            clock_delay();
            swclk_set(0);
        }
        swdio_set(1);
    } else {
        /* Protocol error - no valid ACK */
        /* Turnaround */
        clock_delay();
        swclk_set(1);
        clock_delay();
        swclk_set(0);
        swdio_set_dir_out();
    }

    return (uint8_t)ack;
}

/*
 * Target Reset Control
 */
void probe_set_reset(bool assert)
{
#if USE_DT_GPIO && HAS_RESET_GPIO
    /* Open drain: Active = assert (pull low), Inactive = release (float high) */
    gpio_pin_set_dt(&reset_gpio, assert ? 1 : 0);
    LOG_DBG("Reset %s", assert ? "asserted" : "released");
#elif !USE_DT_GPIO
#ifdef PROBE_RESET_PIN
    if (PROBE_RESET_PIN >= 0) {
        /* Open drain: 0 = assert (pull low), 1 = release (float high) */
        gpio_pin_set(gpio_dev, PROBE_RESET_PIN, assert ? 0 : 1);
        LOG_DBG("Reset %s", assert ? "asserted" : "released");
    }
#endif
#else
    ARG_UNUSED(assert);
#endif
}

/*
 * Enable/disable SWD port
 *
 * On Debug Probe hardware with auto-direction level shifter, there's no
 * explicit enable pin. We just track the connected state.
 */
void probe_port_enable(bool enable)
{
    if (enable) {
        /* Configure pins for active SWD */
        swdio_set_dir_out();
    } else {
        /* Tristate output when disabled */
        swdio_set_dir_in();
    }
    swd_connected = enable;
    LOG_INF("SWD port %s", enable ? "enabled" : "disabled");
}

/*
 * Check if target is connected
 */
bool probe_is_connected(void)
{
    return swd_connected;
}

/*
 * Put SWD pins in high-Z state
 */
void probe_pins_hiz(void)
{
#ifdef CONFIG_SOC_RP2040
    /* When PIO is active, pins are controlled by PIO. We can set SWDIO
     * direction to input via PIO to release the bus, but don't reconfigure
     * the GPIO function which would break PIO control.
     */
    if (use_pio_swd) {
        /* Set SWDIO to input via PIO command */
        pio_swd_read_mode();
        LOG_DBG("Probe pins in Hi-Z (PIO)");
        return;
    }
#endif
#if USE_DT_GPIO
    gpio_pin_configure_dt(&swclk_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&swdio_gpio, GPIO_INPUT);
#if HAS_RESET_GPIO
    gpio_pin_configure_dt(&reset_gpio, GPIO_INPUT);
#endif
#else
    gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_INPUT);
#ifdef PROBE_RESET_PIN
    if (PROBE_RESET_PIN >= 0) {
        gpio_pin_configure(gpio_dev, PROBE_RESET_PIN, GPIO_INPUT);
    }
#endif
#endif
    LOG_DBG("Probe pins in Hi-Z");
}

/*
 * Restore SWD pins to active state
 */
void probe_pins_active(void)
{
#ifdef CONFIG_SOC_RP2040
    /* When PIO is enabled, SWCLK is controlled by PIO side-set and SWDIO
     * direction is controlled by PIO via `out pindirs`. Don't reconfigure.
     */
    if (!use_pio_swd) {
#if USE_DT_GPIO
        gpio_pin_configure_dt(&swclk_gpio, GPIO_OUTPUT_INACTIVE);
#else
        gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_OUTPUT_INACTIVE);
#endif
        swdio_set_dir_out();
    }
#else
#if USE_DT_GPIO
    gpio_pin_configure_dt(&swclk_gpio, GPIO_OUTPUT_INACTIVE);
    swdio_set_dir_out();
#else
    gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_OUTPUT_INACTIVE);
    swdio_set_dir_out();
#endif
#endif
    LOG_DBG("Probe pins active");
}

/*
 * Read a single bit from SWDIO
 */
int probe_read_bit(void)
{
#ifdef CONFIG_SOC_RP2040
    if (use_pio_swd) {
        /* Use PIO for single-bit read when PIO owns the pins */
        return pio_swd_read_bits(1) & 1;
    }
#endif
    int bit;
    swdio_set_dir_in();
    clock_delay();
    swclk_set(1);
    bit = swdio_get();
    clock_delay();
    swclk_set(0);
    return bit;
}

/*
 * Write a single bit to SWDIO
 */
void probe_write_bit(int bit)
{
#ifdef CONFIG_SOC_RP2040
    if (use_pio_swd) {
        /* Use PIO for single-bit write when PIO owns the pins */
        pio_swd_write_bits(1, bit & 1);
        return;
    }
#endif
    swdio_set_dir_out();
    swdio_set(bit);
    clock_delay();
    swclk_set(1);
    clock_delay();
    swclk_set(0);
}

/*
 * JTAG Support
 *
 * JTAG uses the same pins as SWD:
 * - SWCLK -> TCK
 * - SWDIO -> TMS
 * - Additional pins needed: TDI, TDO
 *
 * For basic JTAG support, we bit-bang using available GPIO.
 * Note: Full JTAG requires TDI/TDO pins which may not be available
 * on all debug probe hardware configurations.
 *
 * Note: JTAG support currently uses legacy GPIO path only.
 */

/* JTAG chain configuration */
#define JTAG_MAX_DEVICES 8
static uint8_t jtag_device_count = 0;
static uint8_t jtag_ir_length[JTAG_MAX_DEVICES];
static uint16_t jtag_ir_before[JTAG_MAX_DEVICES];
static uint16_t jtag_ir_after[JTAG_MAX_DEVICES];

/*
 * Initialize JTAG mode
 */
int probe_jtag_init(void)
{
    int ret;

#if USE_DT_GPIO
    /* Configure pins for JTAG mode using devicetree specs */
    /* TCK = SWCLK, TMS = SWDIO */
    ret = gpio_pin_configure_dt(&swclk_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure TCK pin");
        return ret;
    }

    ret = gpio_pin_configure_dt(&swdio_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure TMS pin");
        return ret;
    }

    /* Note: TDI/TDO not yet supported via devicetree */
    LOG_INF("JTAG mode initialized (TDI/TDO not available in DT mode)");
#else
    /* Configure pins for JTAG mode */
    /* TCK = SWCLK, TMS = SWDIO */
    ret = gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure TCK pin");
        return ret;
    }

    ret = gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure TMS pin");
        return ret;
    }

#if PROBE_JTAG_TDI_AVAILABLE
    /* Configure TDI as output (data to target) */
    ret = gpio_pin_configure(gpio_dev, PROBE_TDI_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure TDI pin");
        return ret;
    }
    LOG_DBG("TDI pin configured (GP%d)", PROBE_TDI_PIN);
#endif

#if PROBE_JTAG_TDO_AVAILABLE
    /* Configure TDO as input (data from target) */
    ret = gpio_pin_configure(gpio_dev, PROBE_TDO_PIN, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure TDO pin");
        return ret;
    }
    LOG_DBG("TDO pin configured (GP%d)", PROBE_TDO_PIN);
#endif

    LOG_INF("JTAG mode initialized (TDI/TDO %s)",
            PROBE_JTAG_FULL_SUPPORT ? "available" : "not available");
#endif

    jtag_device_count = 0;
    memset(jtag_ir_length, 0, sizeof(jtag_ir_length));

    return 0;
}

/*
 * Deinitialize JTAG mode
 */
void probe_jtag_deinit(void)
{
#if USE_DT_GPIO
    /* Put JTAG pins in high-Z state */
    gpio_pin_configure_dt(&swclk_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&swdio_gpio, GPIO_INPUT);
#else
    /* Put JTAG pins in high-Z state */
    gpio_pin_configure(gpio_dev, PROBE_SWCLK_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_INPUT);

#if PROBE_JTAG_TDI_AVAILABLE
    gpio_pin_configure(gpio_dev, PROBE_TDI_PIN, GPIO_INPUT);
#endif

#if PROBE_JTAG_TDO_AVAILABLE
    gpio_pin_configure(gpio_dev, PROBE_TDO_PIN, GPIO_INPUT);
#endif
#endif

    LOG_INF("JTAG mode deinitialized");
}

/*
 * Check if full JTAG is supported (TDI/TDO available)
 */
bool probe_jtag_available(void)
{
#if USE_DT_GPIO
    /* JTAG TDI/TDO not yet supported via devicetree */
    return false;
#else
    return PROBE_JTAG_FULL_SUPPORT;
#endif
}

/*
 * Configure JTAG chain
 */
void probe_jtag_configure(uint8_t count, const uint8_t *ir_length)
{
    uint16_t bits_before = 0;
    uint16_t bits_after = 0;

    if (count > JTAG_MAX_DEVICES) {
        count = JTAG_MAX_DEVICES;
    }

    jtag_device_count = count;

    /* Calculate IR bits before and after each device */
    for (int i = 0; i < count; i++) {
        jtag_ir_length[i] = ir_length[i];
        jtag_ir_before[i] = bits_before;
        bits_before += ir_length[i];
    }

    bits_after = bits_before;
    for (int i = 0; i < count; i++) {
        bits_after -= jtag_ir_length[i];
        jtag_ir_after[i] = bits_after;
    }

    LOG_INF("JTAG chain configured: %d devices", count);
}

/*
 * Clock TCK with TMS value (no TDI/TDO)
 */
static inline void jtag_tck_cycle(int tms)
{
    swdio_set(tms);  /* TMS = SWDIO */
    clock_delay();
    swclk_set(1);    /* TCK = SWCLK */
    clock_delay();
    swclk_set(0);
}

/*
 * Clock TCK with TMS and TDI, return TDO
 * TDI is sampled on rising edge, TDO changes on falling edge
 */
static inline int jtag_tck_cycle_tdi_tdo(int tms, int tdi_val)
{
    int tdo_val;

    /* Set TMS and TDI before rising edge */
    swdio_set(tms);          /* TMS = SWDIO */
    tdi_set(tdi_val);        /* TDI output */
    clock_delay();

    /* Rising edge - target samples TDI */
    swclk_set(1);

    /* Read TDO (stable during high phase) */
    tdo_val = tdo_get();
    clock_delay();

    /* Falling edge - target updates TDO */
    swclk_set(0);

    return tdo_val;
}

/*
 * Execute JTAG sequence
 *
 * Executes a sequence of JTAG clocks with specified TMS, TDI data,
 * and optionally captures TDO data.
 *
 * @param info Sequence info byte:
 *             - Bits 0-5: TCK count (0 = 64 clocks)
 *             - Bit 6: TMS value during sequence
 *             - Bit 7: TDO capture enable
 * @param tdi  TDI data to output (LSB first), may be NULL
 * @param tdo  Buffer for captured TDO data (if bit 7 set), may be NULL
 */
void probe_jtag_sequence(uint8_t info, const uint8_t *tdi, uint8_t *tdo)
{
    uint32_t count = info & 0x3F;
    int tms = (info >> 6) & 1;
    int capture_tdo = (info >> 7) & 1;
    uint32_t n;
    uint32_t byte_idx;
    uint32_t bit_idx;
    int tdi_val;
    int tdo_val;
    uint8_t tdo_byte = 0;

    if (count == 0) {
        count = 64;
    }

    /* TMS is always output */
    swdio_set_dir_out();

    for (n = 0; n < count; n++) {
        byte_idx = n / 8;
        bit_idx = n % 8;

        /* Get TDI bit (LSB first) */
        if (tdi != NULL) {
            tdi_val = (tdi[byte_idx] >> bit_idx) & 1;
        } else {
            tdi_val = 0;
        }

        /* Clock cycle with TDI/TDO */
        tdo_val = jtag_tck_cycle_tdi_tdo(tms, tdi_val);

        /* Capture TDO bit (LSB first) */
        if (capture_tdo) {
            if (bit_idx == 0) {
                tdo_byte = 0;
            }
            tdo_byte |= (tdo_val << bit_idx);

            /* Store complete byte or last partial byte */
            if (tdo != NULL && (bit_idx == 7 || n == count - 1)) {
                tdo[byte_idx] = tdo_byte;
            }
        }
    }
}

/*
 * Read JTAG IDCODE
 *
 * Reads the 32-bit IDCODE from a device in the JTAG chain.
 * After reset, the IDCODE register is selected by default.
 */
int probe_jtag_read_idcode(uint8_t index, uint32_t *idcode)
{
    uint32_t data = 0;
    int tdo_val;
    int n;

    if (index >= jtag_device_count && jtag_device_count > 0) {
        return -EINVAL;
    }

    /* Move to Test-Logic-Reset (5 TCK with TMS=1) */
    for (int i = 0; i < 5; i++) {
        jtag_tck_cycle(1);
    }

    /* Move to Run-Test/Idle */
    jtag_tck_cycle(0);

    /* Move to Select-DR-Scan */
    jtag_tck_cycle(1);

    /* Move to Capture-DR */
    jtag_tck_cycle(0);

    /* Move to Shift-DR */
    jtag_tck_cycle(0);

    /* Skip devices before target in chain (shift their bypass bits) */
    for (n = 0; n < (int)index; n++) {
        jtag_tck_cycle_tdi_tdo(0, 0);
    }

    /* Shift out IDCODE (32 bits) with TMS=0, TDI=0 */
    for (n = 0; n < 32; n++) {
        /* TMS=1 on last bit to exit Shift-DR */
        int tms = (n == 31) ? 1 : 0;
        tdo_val = jtag_tck_cycle_tdi_tdo(tms, 0);
        data |= ((uint32_t)tdo_val << n);
    }

    *idcode = data;

    /* Now in Exit1-DR, move to Update-DR */
    jtag_tck_cycle(1);

    /* Move to Run-Test/Idle */
    jtag_tck_cycle(0);

    LOG_DBG("JTAG IDCODE read for device %d: 0x%08x", index, data);
    return 0;
}

/*
 * Write JTAG IR (Instruction Register)
 *
 * Shifts an instruction into the IR of a specific device in the chain.
 */
int probe_jtag_write_ir(uint8_t index, uint32_t ir, uint8_t ir_len)
{
    int n;
    int bit;

    if (index >= jtag_device_count && jtag_device_count > 0) {
        return -EINVAL;
    }

    /* Move to Test-Logic-Reset (5 TCK with TMS=1) */
    for (int i = 0; i < 5; i++) {
        jtag_tck_cycle(1);
    }

    /* Move to Run-Test/Idle */
    jtag_tck_cycle(0);

    /* Move to Select-DR-Scan */
    jtag_tck_cycle(1);

    /* Move to Select-IR-Scan */
    jtag_tck_cycle(1);

    /* Move to Capture-IR */
    jtag_tck_cycle(0);

    /* Move to Shift-IR */
    jtag_tck_cycle(0);

    /* Shift BYPASS (all 1s) to devices before target */
    for (n = 0; n < jtag_ir_before[index]; n++) {
        jtag_tck_cycle_tdi_tdo(0, 1);
    }

    /* Shift IR value to target device */
    for (n = 0; n < ir_len; n++) {
        bit = (ir >> n) & 1;
        /* TMS=1 on last bit if no devices after, else TMS=0 */
        int tms = (n == ir_len - 1 && jtag_ir_after[index] == 0) ? 1 : 0;
        jtag_tck_cycle_tdi_tdo(tms, bit);
    }

    /* Shift BYPASS (all 1s) to devices after target */
    for (n = 0; n < jtag_ir_after[index]; n++) {
        int tms = (n == jtag_ir_after[index] - 1) ? 1 : 0;
        jtag_tck_cycle_tdi_tdo(tms, 1);
    }

    /* Now in Exit1-IR, move to Update-IR */
    jtag_tck_cycle(1);

    /* Move to Run-Test/Idle */
    jtag_tck_cycle(0);

    LOG_DBG("JTAG IR write for device %d: 0x%x (%d bits)", index, ir, ir_len);
    return 0;
}

/*
 * Transfer JTAG DR (Data Register)
 *
 * Shifts data in/out of the DR of a specific device in the chain.
 * Other devices in the chain are assumed to be in BYPASS mode.
 */
int probe_jtag_transfer_dr(uint8_t index, const uint32_t *dr_out, uint32_t *dr_in,
                           uint8_t dr_len)
{
    int n;
    int bit;
    int tdo_val;
    uint32_t data_in = 0;

    if (index >= jtag_device_count && jtag_device_count > 0) {
        return -EINVAL;
    }

    /* Move to Select-DR-Scan (from Run-Test/Idle) */
    jtag_tck_cycle(1);

    /* Move to Capture-DR */
    jtag_tck_cycle(0);

    /* Move to Shift-DR */
    jtag_tck_cycle(0);

    /* Shift bypass bits for devices before target (1 bit each) */
    for (n = 0; n < (int)index; n++) {
        jtag_tck_cycle_tdi_tdo(0, 0);
    }

    /* Shift DR data to/from target device */
    for (n = 0; n < dr_len; n++) {
        /* Get output bit */
        if (dr_out != NULL) {
            bit = (*dr_out >> n) & 1;
        } else {
            bit = 0;
        }

        /* Check if this is the last bit overall */
        int devices_after = jtag_device_count > 0 ? (jtag_device_count - index - 1) : 0;
        int tms = (n == dr_len - 1 && devices_after == 0) ? 1 : 0;

        tdo_val = jtag_tck_cycle_tdi_tdo(tms, bit);

        /* Capture input bit */
        if (dr_in != NULL) {
            data_in |= ((uint32_t)tdo_val << n);
        }
    }

    /* Shift bypass bits for devices after target */
    int devices_after = jtag_device_count > 0 ? (jtag_device_count - index - 1) : 0;
    for (n = 0; n < devices_after; n++) {
        int tms = (n == devices_after - 1) ? 1 : 0;
        jtag_tck_cycle_tdi_tdo(tms, 0);
    }

    /* Return captured data */
    if (dr_in != NULL) {
        *dr_in = data_in;
    }

    /* Now in Exit1-DR, move to Update-DR */
    jtag_tck_cycle(1);

    /* Move to Run-Test/Idle */
    jtag_tck_cycle(0);

    LOG_DBG("JTAG DR transfer for device %d: %d bits", index, dr_len);
    return 0;
}

/*
 * Diagnostic functions for shell commands
 */

#if USE_DT_GPIO
/*
 * Get SWD pin states
 */
void probe_get_pin_state(int *swclk, int *swdio, int *reset)
{
    if (swclk) {
        *swclk = gpio_pin_get_dt(&swclk_gpio);
    }
    if (swdio) {
        *swdio = gpio_pin_get_dt(&swdio_gpio);
    }
    if (reset) {
#if HAS_RESET_GPIO
        *reset = gpio_pin_get_dt(&reset_gpio);
#else
        *reset = -1;
#endif
    }
}

/*
 * Get SWD pin numbers for display
 */
void probe_get_pin_info(int *swclk_pin, int *swdio_pin, int *reset_pin)
{
    if (swclk_pin) {
        *swclk_pin = swclk_gpio.pin;
    }
    if (swdio_pin) {
        *swdio_pin = swdio_gpio.pin;
    }
    if (reset_pin) {
#if HAS_RESET_GPIO
        *reset_pin = reset_gpio.pin;
#else
        *reset_pin = -1;
#endif
    }
}
#else
void probe_get_pin_state(int *swclk, int *swdio, int *reset)
{
    if (swclk) *swclk = -1;
    if (swdio) *swdio = -1;
    if (reset) *reset = -1;
}

void probe_get_pin_info(int *swclk_pin, int *swdio_pin, int *reset_pin)
{
    if (swclk_pin) *swclk_pin = PROBE_SWCLK_PIN;
    if (swdio_pin) *swdio_pin = PROBE_SWDIO_PIN;
    if (reset_pin) *reset_pin = PROBE_RESET_PIN;
}
#endif

/*
 * Check if PIO acceleration is in use
 */
bool probe_is_pio_enabled(void)
{
#ifdef CONFIG_SOC_RP2040
    return use_pio_swd;
#else
    return false;
#endif
}

/*
 * Enable/disable PIO acceleration (for debugging)
 */
void probe_set_pio_enabled(bool enable)
{
#ifdef CONFIG_SOC_RP2040
    if (enable && !use_pio_swd) {
        /* Re-initialize PIO */
        if (pio_swd_init() == 0) {
            use_pio_swd = true;
            LOG_INF("PIO SWD re-enabled");
        }
    } else if (!enable && use_pio_swd) {
        /* Switch to GPIO mode */
        pio_swd_deinit();
        use_pio_swd = false;
        /* Re-init GPIO pins for bit-bang using SDK calls to ensure SIO function */
        gpio_set_function(PROBE_SWCLK_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(PROBE_SWCLK_PIN, GPIO_OUT);
        gpio_put(PROBE_SWCLK_PIN, 0);
        gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(PROBE_SWDIO_PIN, GPIO_OUT);
        gpio_put(PROBE_SWDIO_PIN, 0);
        /* Ensure SWDI is configured as input for reading from target */
#if defined(PROBE_SWDI_PIN) && (PROBE_SWDI_PIN >= 0)
        gpio_init(PROBE_SWDI_PIN);
        gpio_set_dir(PROBE_SWDI_PIN, GPIO_IN);
        gpio_pull_up(PROBE_SWDI_PIN);
#endif
        LOG_INF("PIO SWD disabled, using GPIO bit-bang");
    }
#else
    ARG_UNUSED(enable);
#endif
}

/*
 * Get system clock frequency
 */
uint32_t probe_get_sys_clock(void)
{
    return sys_clock_hw_cycles_per_sec();
}
