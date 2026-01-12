/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Autobaud Detection Implementation
 *
 * Automatic baud rate detection using PIO edge timing measurement.
 * Port of autobaud.c from the original debugprobe.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_SOC_RP2040

#include <hardware/pio.h>
#include <hardware/clocks.h>

#include "autobaud.h"

LOG_MODULE_REGISTER(autobaud, CONFIG_LOG_DEFAULT_LEVEL);

/* PIO instance and state machine (use PIO1 to not conflict with SWD) */
#define AUTOBAUD_PIO    pio1
#define AUTOBAUD_SM     0

/* Use system clock frequency from platform_defs.h (SYS_CLK_HZ) */

/* Minimum number of samples for valid detection */
#define MIN_SAMPLES     50

/* Tolerance for grouping similar timing values (10%) */
#define TIMING_TOLERANCE_PERCENT    10

/* Minimum validity score to report result */
#define MIN_VALIDITY    0.6f

/* Sample buffer size - larger for better accuracy */
#define AUTOBAUD_SAMPLE_COUNT   256

/* State tracking */
static bool autobaud_running = false;
static uint32_t pio_program_offset = 0;
static uint8_t autobaud_rx_pin = 0;

/* Detection state - larger buffer for better accuracy */
static struct {
    uint32_t samples[AUTOBAUD_SAMPLE_COUNT];
    uint32_t sample_count;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint32_t detected_baud;
    float validity;
    /* Histogram for frequency analysis */
    uint32_t histogram[16];
    uint32_t hist_min;
    uint32_t hist_step;
} detect_state;

/* Semaphore for result notification */
static K_SEM_DEFINE(result_sem, 0, 1);

/*
 * Process a raw timing sample from PIO
 *
 * The PIO counts down from 0xFFFFFFFF, so actual cycles = UINT32_MAX - raw
 * Each count represents 2 clock cycles (jmp + conditional jmp)
 */
static void process_sample(uint32_t raw)
{
    uint32_t cycles;

    /* Convert raw counter to cycle count */
    cycles = (UINT32_MAX - raw) * 2;

    /* Ignore very short or very long pulses (noise)
     * Min: ~500 cycles at 125MHz = 250kbaud (way above typical)
     * Max: reasonable for slowest baud rates
     */
    if (cycles < 100 || cycles > SYS_CLK_HZ / 100) {
        return;
    }

    /* Track min/max (for histogram setup) */
    if (detect_state.sample_count == 0) {
        detect_state.min_cycles = cycles;
        detect_state.max_cycles = cycles;
    } else {
        if (cycles < detect_state.min_cycles) {
            detect_state.min_cycles = cycles;
        }
        if (cycles > detect_state.max_cycles) {
            detect_state.max_cycles = cycles;
        }
    }

    /* Store sample in circular manner */
    uint32_t idx = detect_state.sample_count % AUTOBAUD_SAMPLE_COUNT;
    detect_state.samples[idx] = cycles;
    detect_state.sample_count++;
}

/*
 * Build histogram of sample timing values
 */
static void build_histogram(void)
{
    uint32_t range = detect_state.max_cycles - detect_state.min_cycles;
    uint32_t num_samples = MIN(detect_state.sample_count, AUTOBAUD_SAMPLE_COUNT);

    /* Clear histogram */
    memset(detect_state.histogram, 0, sizeof(detect_state.histogram));

    if (range == 0) {
        range = 1;
    }

    detect_state.hist_min = detect_state.min_cycles;
    detect_state.hist_step = (range + 15) / 16;  /* 16 bins */

    /* Populate histogram */
    for (uint32_t i = 0; i < num_samples; i++) {
        uint32_t s = detect_state.samples[i];
        uint32_t bin = (s - detect_state.hist_min) / detect_state.hist_step;
        if (bin >= 16) bin = 15;
        detect_state.histogram[bin]++;
    }
}

/*
 * Find the peak in histogram (most common timing value)
 */
static uint32_t find_histogram_peak(void)
{
    uint32_t max_count = 0;
    uint32_t peak_bin = 0;

    for (int i = 0; i < 16; i++) {
        if (detect_state.histogram[i] > max_count) {
            max_count = detect_state.histogram[i];
            peak_bin = i;
        }
    }

    /* Return center of peak bin */
    return detect_state.hist_min + (peak_bin * detect_state.hist_step) +
           (detect_state.hist_step / 2);
}

/*
 * Analyze collected samples to determine baud rate
 * Uses histogram-based analysis for robustness
 */
static void analyze_samples(void)
{
    uint32_t sum = 0;
    uint32_t count = 0;
    uint32_t avg_cycles;
    uint32_t baud;
    float validity;
    uint32_t num_samples = MIN(detect_state.sample_count, AUTOBAUD_SAMPLE_COUNT);

    if (detect_state.sample_count < MIN_SAMPLES) {
        return;
    }

    /* Build histogram to find most common timing */
    build_histogram();

    /* Find the peak timing value (most likely 1-bit period) */
    uint32_t peak_cycles = find_histogram_peak();
    uint32_t tolerance = peak_cycles * TIMING_TOLERANCE_PERCENT / 100;

    /* Sum samples near the peak */
    for (uint32_t i = 0; i < num_samples; i++) {
        uint32_t s = detect_state.samples[i];
        if (s >= peak_cycles - tolerance && s <= peak_cycles + tolerance) {
            sum += s;
            count++;
        }
    }

    if (count < MIN_SAMPLES / 2) {
        return;
    }

    /* Calculate average 1-bit period */
    avg_cycles = sum / count;

    /* Calculate baud rate: baud = clock_freq / cycles_per_bit */
    baud = SYS_CLK_HZ / avg_cycles;

    /* Round to nearest standard baud rate */
    static const uint32_t standard_bauds[] = {
        300, 1200, 2400, 4800, 9600, 14400, 19200, 28800,
        38400, 57600, 76800, 115200, 230400, 460800, 921600,
        1000000, 1500000, 2000000, 3000000
    };

    uint32_t closest_baud = baud;
    uint32_t min_diff = UINT32_MAX;

    for (size_t i = 0; i < ARRAY_SIZE(standard_bauds); i++) {
        uint32_t diff = (baud > standard_bauds[i]) ?
                        (baud - standard_bauds[i]) :
                        (standard_bauds[i] - baud);
        if (diff < min_diff) {
            min_diff = diff;
            closest_baud = standard_bauds[i];
        }
    }

    /* Calculate validity score */
    /* Higher sample count and closer to standard baud = higher validity */
    float sample_score = (float)count / (float)num_samples;
    float baud_error = (float)min_diff / (float)baud;
    validity = sample_score * (1.0f - MIN(baud_error, 0.5f));

    if (validity > MIN_VALIDITY && validity > detect_state.validity) {
        detect_state.detected_baud = closest_baud;
        detect_state.validity = validity;
        k_sem_give(&result_sem);
        LOG_INF("Autobaud detected: %u (validity: %.2f, samples: %u)",
                closest_baud, (double)validity, count);
    }
}

/*
 * Initialize PIO state machine for autobaud
 */
static void autobaud_sm_init(uint8_t rx_pin)
{
    PIO pio = AUTOBAUD_PIO;
    uint sm = AUTOBAUD_SM;

    /* Configure state machine */
    pio_sm_config c = pio_get_default_sm_config();

    /* Set input pin */
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_jmp_pin(&c, rx_pin);

    /* IN shifts right, autopush at 32 bits */
    sm_config_set_in_shift(&c, true, true, 32);

    /* Run at system clock (no divider) for maximum timing resolution */
    sm_config_set_clkdiv(&c, 1.0f);

    /* Initialize GPIO for PIO use */
    pio_gpio_init(pio, rx_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, rx_pin, 1, false);  /* Input */

    /* Initialize and enable state machine */
    pio_sm_init(pio, sm, pio_program_offset + PIO_AUTOBAUD_WRAP_TARGET, &c);
    pio_sm_set_enabled(pio, sm, true);
}

/*
 * Autobaud detection thread
 */
static void autobaud_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    PIO pio = AUTOBAUD_PIO;
    uint sm = AUTOBAUD_SM;

    LOG_INF("Autobaud thread started");

    while (autobaud_running) {
        /* Check for samples in PIO FIFO */
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t raw = pio_sm_get_blocking(pio, sm);
            process_sample(raw);
        }

        /* Periodically analyze collected samples */
        if (detect_state.sample_count >= MIN_SAMPLES) {
            analyze_samples();
        }

        /* Short sleep to yield CPU */
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Autobaud thread stopped");
}

K_THREAD_STACK_DEFINE(autobaud_stack, 1024);
static struct k_thread autobaud_thread;

/*
 * Start autobaud detection
 */
int autobaud_start(uint8_t rx_pin)
{
    PIO pio = AUTOBAUD_PIO;

    if (autobaud_running) {
        return -EALREADY;
    }

    /* Reset detection state */
    memset(&detect_state, 0, sizeof(detect_state));
    k_sem_reset(&result_sem);
    autobaud_rx_pin = rx_pin;

    /* Check if we can add the program */
    if (!pio_can_add_program(pio, &(pio_program_t){
        .instructions = pio_autobaud_instructions,
        .length = PIO_AUTOBAUD_PROGRAM_LENGTH,
        .origin = -1
    })) {
        LOG_ERR("Cannot add autobaud PIO program");
        return -ENOMEM;
    }

    /* Add program to PIO */
    pio_program_offset = pio_add_program(pio, &(pio_program_t){
        .instructions = pio_autobaud_instructions,
        .length = PIO_AUTOBAUD_PROGRAM_LENGTH,
        .origin = -1
    });

    /* Initialize state machine */
    autobaud_sm_init(rx_pin);

    autobaud_running = true;

    /* Start detection thread */
    k_thread_create(&autobaud_thread, autobaud_stack,
                    K_THREAD_STACK_SIZEOF(autobaud_stack),
                    autobaud_thread_entry, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);

    LOG_INF("Autobaud started on pin %u", rx_pin);

    return 0;
}

/*
 * Stop autobaud detection
 */
void autobaud_stop(void)
{
    if (!autobaud_running) {
        return;
    }

    PIO pio = AUTOBAUD_PIO;
    uint sm = AUTOBAUD_SM;

    autobaud_running = false;

    /* Wait for thread to exit */
    k_thread_join(&autobaud_thread, K_MSEC(100));

    /* Disable state machine */
    pio_sm_set_enabled(pio, sm, false);

    /* Remove program */
    pio_remove_program(pio, &(pio_program_t){
        .instructions = pio_autobaud_instructions,
        .length = PIO_AUTOBAUD_PROGRAM_LENGTH,
        .origin = pio_program_offset
    }, pio_program_offset);

    /* Reset GPIO to regular mode */
    gpio_set_function(autobaud_rx_pin, GPIO_FUNC_SIO);

    LOG_INF("Autobaud stopped");
}

/*
 * Check if autobaud is running
 */
bool autobaud_is_running(void)
{
    return autobaud_running;
}

/*
 * Get detected baud rate (non-blocking)
 */
bool autobaud_get_result(struct autobaud_result *result)
{
    if (detect_state.detected_baud == 0) {
        return false;
    }

    if (result) {
        result->baud = detect_state.detected_baud;
        result->validity = detect_state.validity;
    }

    return true;
}

/*
 * Wait for baud rate detection
 */
int autobaud_wait(struct autobaud_result *result, uint32_t timeout_ms)
{
    int ret = k_sem_take(&result_sem, K_MSEC(timeout_ms));

    if (ret == 0) {
        if (result) {
            result->baud = detect_state.detected_baud;
            result->validity = detect_state.validity;
        }
        return 0;
    }

    return -ETIMEDOUT;
}

#endif /* CONFIG_SOC_RP2040 */
