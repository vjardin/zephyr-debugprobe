/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - Autobaud Detection Header
 *
 * Automatic baud rate detection using PIO edge timing measurement.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_SOC_RP2040

/*
 * Autobaud PIO Program - Pre-assembled instructions
 *
 * Original assembly (autobaud.pio):
 *
 * .program autobaud
 *
 * .wrap_target
 * falling_edge:
 *     wait 0 pin 0                    ; falling edge detected
 *     set x, 0                        ; set cycle counter to 0
 *     mov x, ~x                       ; x = 0xFFFFFFFF
 *
 * count_cycles:
 *     jmp pin rising_edge             ; if line went high, exit loop
 *     jmp x-- count_cycles            ; otherwise, decrement and continue
 *
 * rising_edge:
 *     mov isr, x                      ; save elapsed cycle count
 *     push noblock                    ; push to FIFO
 *     jmp falling_edge                ; repeat
 * .wrap
 */

/* Pre-assembled PIO instructions for autobaud */
static const uint16_t pio_autobaud_instructions[] = {
    /* 0: wait 0 pin 0                 - falling_edge (wrap_target) */
    0x2020,
    /* 1: set x, 0                     */
    0xe020,
    /* 2: mov x, ~x                    - x = 0xFFFFFFFF */
    0xa02f,
    /* 3: jmp pin 5                    - count_cycles: jmp to rising_edge */
    0x00c5,
    /* 4: jmp x-- 3                    - continue counting */
    0x0043,
    /* 5: mov isr, x                   - rising_edge */
    0xa0c1,
    /* 6: push noblock                 */
    0x8000,
    /* 7: jmp 0                        - back to falling_edge */
    0x0000,
};

#define PIO_AUTOBAUD_PROGRAM_LENGTH  ARRAY_SIZE(pio_autobaud_instructions)
#define PIO_AUTOBAUD_WRAP_TARGET     0
#define PIO_AUTOBAUD_WRAP            7

/* Baud rate detection result */
struct autobaud_result {
    uint32_t baud;      /* Detected baud rate */
    float validity;     /* Confidence score (0.0-1.0) */
};

/**
 * @brief Start autobaud detection
 *
 * Starts the PIO state machine to measure edge timing on the UART RX pin.
 * The detection runs in the background.
 *
 * @param rx_pin GPIO pin number for UART RX
 * @return 0 on success, negative error code otherwise
 */
int autobaud_start(uint8_t rx_pin);

/**
 * @brief Stop autobaud detection
 */
void autobaud_stop(void);

/**
 * @brief Check if autobaud is running
 *
 * @return true if detection is active
 */
bool autobaud_is_running(void);

/**
 * @brief Get detected baud rate
 *
 * Non-blocking call to check if a baud rate has been detected.
 *
 * @param result Pointer to store detection result
 * @return true if a valid baud rate was detected
 */
bool autobaud_get_result(struct autobaud_result *result);

/**
 * @brief Wait for baud rate detection
 *
 * Blocking call that waits until a valid baud rate is detected
 * or timeout expires.
 *
 * @param result Pointer to store detection result
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return 0 on success, -ETIMEDOUT on timeout
 */
int autobaud_wait(struct autobaud_result *result, uint32_t timeout_ms);

#endif /* CONFIG_SOC_RP2040 */
