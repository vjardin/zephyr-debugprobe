/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - SWD I/O Implementation
 *
 * Provides both GPIO bit-bang and PIO-based SWD implementations.
 * The PIO version is for RP2040/RP2350 and provides much higher performance.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "probe_config.h"
#include "swdio.h"

LOG_MODULE_REGISTER(swdio, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(CONFIG_SOC_SERIES_RP2XXX) || defined(CONFIG_SOC_FAMILY_RPI_PICO)
/* 
 * RP2040/RP2350 PIO-based implementation
 * This provides much higher SWD clock rates than GPIO bit-banging
 */

#include <hardware/pio.h>
#include <hardware/clocks.h>

/* PIO programs for SWD */
/* These would typically be generated from .pio files */

/*
 * SWD Write PIO program
 * Side-set: SWCLK
 * OUT: SWDIO
 */
static const uint16_t swd_write_program[] = {
    /* 0: out pins, 1 side 0 [1]  ; Output bit, clock low */
    0x6001,
    /* 1: nop        side 1 [1]  ; Clock high */
    0xA042,
    /* 2: jmp !osre, 0            ; Loop while data */
    0x00C0,
};

/*
 * SWD Read PIO program
 * Side-set: SWCLK
 * IN: SWDIO
 */
static const uint16_t swd_read_program[] = {
    /* 0: nop        side 1 [1]  ; Clock high */
    0xA042,
    /* 1: in pins, 1 side 0 [1]  ; Sample bit, clock low */
    0x4001,
    /* 2: jmp y--, 0             ; Loop while count */
    0x0080,
};

/* PIO state */
static PIO pio_hw = pio0;
static uint sm_write = 0;
static uint sm_read = 1;
static bool pio_initialized = false;

/*
 * Initialize PIO for SWD
 */
int swdio_pio_init(void)
{
    uint offset_write, offset_read;

    if (pio_initialized) {
        return 0;
    }

    /* Claim state machines */
    sm_write = pio_claim_unused_sm(pio_hw, true);
    sm_read = pio_claim_unused_sm(pio_hw, true);

    if (sm_write < 0 || sm_read < 0) {
        LOG_ERR("Failed to claim PIO state machines");
        return -ENODEV;
    }

    /* Add programs to PIO memory */
    if (!pio_can_add_program(pio_hw, &(pio_program_t){
            .instructions = swd_write_program,
            .length = ARRAY_SIZE(swd_write_program),
            .origin = -1
        })) {
        LOG_ERR("No space for SWD write program");
        return -ENOMEM;
    }

    offset_write = pio_add_program(pio_hw, &(pio_program_t){
        .instructions = swd_write_program,
        .length = ARRAY_SIZE(swd_write_program),
        .origin = -1
    });

    offset_read = pio_add_program(pio_hw, &(pio_program_t){
        .instructions = swd_read_program,
        .length = ARRAY_SIZE(swd_read_program),
        .origin = -1
    });

    /* Configure write SM */
    pio_sm_config cfg_write = pio_get_default_sm_config();
    sm_config_set_wrap(&cfg_write, offset_write, offset_write + ARRAY_SIZE(swd_write_program) - 1);
    sm_config_set_sideset(&cfg_write, 1, false, false);
    sm_config_set_sideset_pins(&cfg_write, PROBE_SWCLK_PIN);
    sm_config_set_out_pins(&cfg_write, PROBE_SWDIO_PIN, 1);
    sm_config_set_set_pins(&cfg_write, PROBE_SWDIO_PIN, 1);
    sm_config_set_out_shift(&cfg_write, true, true, 32); /* LSB first, autopull */
    
    pio_gpio_init(pio_hw, PROBE_SWCLK_PIN);
    pio_gpio_init(pio_hw, PROBE_SWDIO_PIN);
    
    pio_sm_set_consecutive_pindirs(pio_hw, sm_write, PROBE_SWCLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio_hw, sm_write, PROBE_SWDIO_PIN, 1, true);
    
    pio_sm_init(pio_hw, sm_write, offset_write, &cfg_write);

    /* Configure read SM */
    pio_sm_config cfg_read = pio_get_default_sm_config();
    sm_config_set_wrap(&cfg_read, offset_read, offset_read + ARRAY_SIZE(swd_read_program) - 1);
    sm_config_set_sideset(&cfg_read, 1, false, false);
    sm_config_set_sideset_pins(&cfg_read, PROBE_SWCLK_PIN);
    sm_config_set_in_pins(&cfg_read, PROBE_SWDIO_PIN);
    sm_config_set_in_shift(&cfg_read, true, true, 32); /* LSB first, autopush */
    
    pio_sm_init(pio_hw, sm_read, offset_read, &cfg_read);

    pio_initialized = true;
    LOG_INF("PIO SWD initialized");

    return 0;
}

/*
 * Set PIO clock divider for SWD frequency
 */
void swdio_pio_set_clock(uint32_t freq_hz)
{
    /* Clock divider = sys_clk / (freq_hz * cycles_per_bit) */
    float div = (float)clock_get_hz(clk_sys) / (float)(freq_hz * 4);
    
    if (div < 1.0f) {
        div = 1.0f;
    }

    pio_sm_set_clkdiv(pio_hw, sm_write, div);
    pio_sm_set_clkdiv(pio_hw, sm_read, div);

    LOG_DBG("PIO clock div: %.2f for %u Hz", (double)div, freq_hz);
}

/*
 * PIO-based SWD write
 */
void swdio_pio_write(uint32_t data, uint32_t bits)
{
    /* Enable write SM */
    pio_sm_set_enabled(pio_hw, sm_write, true);
    pio_sm_set_consecutive_pindirs(pio_hw, sm_write, PROBE_SWDIO_PIN, 1, true);

    /* Set bit count and push data */
    pio_sm_put_blocking(pio_hw, sm_write, bits - 1);
    pio_sm_put_blocking(pio_hw, sm_write, data);

    /* Wait for completion */
    while (!pio_sm_is_tx_fifo_empty(pio_hw, sm_write)) {
        tight_loop_contents();
    }

    pio_sm_set_enabled(pio_hw, sm_write, false);
}

/*
 * PIO-based SWD read
 */
uint32_t swdio_pio_read(uint32_t bits)
{
    uint32_t data;

    /* Set SWDIO as input */
    pio_sm_set_consecutive_pindirs(pio_hw, sm_read, PROBE_SWDIO_PIN, 1, false);

    /* Enable read SM */
    pio_sm_set_enabled(pio_hw, sm_read, true);

    /* Set bit count */
    pio_sm_put_blocking(pio_hw, sm_read, bits - 1);

    /* Wait for data and read */
    data = pio_sm_get_blocking(pio_hw, sm_read);

    pio_sm_set_enabled(pio_hw, sm_read, false);

    /* Mask to actual bit count */
    if (bits < 32) {
        data &= (1U << bits) - 1;
    }

    return data;
}

#else /* Non-PIO platforms */

/*
 * Stub implementations for platforms without PIO
 */

int swdio_pio_init(void)
{
    LOG_INF("PIO not available, using GPIO bit-bang");
    return 0;
}

void swdio_pio_set_clock(uint32_t freq_hz)
{
    ARG_UNUSED(freq_hz);
    /* Clock is controlled by software delays in GPIO mode */
}

void swdio_pio_write(uint32_t data, uint32_t bits)
{
    ARG_UNUSED(data);
    ARG_UNUSED(bits);
    /* Use GPIO bit-bang implementation in probe.c */
}

uint32_t swdio_pio_read(uint32_t bits)
{
    ARG_UNUSED(bits);
    /* Use GPIO bit-bang implementation in probe.c */
    return 0;
}

#endif /* PIO support */

/*
 * Check if PIO is available and initialized
 */
bool swdio_pio_available(void)
{
#if defined(CONFIG_SOC_SERIES_RP2XXX) || defined(CONFIG_SOC_FAMILY_RPI_PICO)
    return pio_initialized;
#else
    return false;
#endif
}
