/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - PIO-based SWD Header
 *
 * High-speed SWD implementation using RP2040 PIO state machines.
 * Port of probe.pio from the original debugprobe.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_SOC_RP2040

#include <hardware/pio.h>
#include "probe_config.h"

/*
 * PIO SWD Program
 *
 * If PIO_GENERATED_HEADERS is defined, use the pioasm-generated header.
 * Otherwise, use pre-assembled instructions as fallback.
 */
#ifdef PIO_GENERATED_HEADERS
/* Use pioasm-generated headers */
#include "probe.pio.h"

/* Map generated names to our internal names */
#define pio_swd_program             probe_program
#define pio_swd_program_instructions probe_program_instructions
#define PIO_SWD_PROGRAM_LENGTH      (sizeof(probe_program_instructions) / sizeof(probe_program_instructions[0]))
#define PIO_SWD_WRAP_TARGET         probe_wrap_target
#define PIO_SWD_WRAP                probe_wrap

/* Entry points from generated header */
#define PIO_SWD_OFFSET_WRITE_CMD      probe_offset_write_cmd
#define PIO_SWD_OFFSET_TURNAROUND_CMD probe_offset_turnaround_cmd
#define PIO_SWD_OFFSET_GET_NEXT_CMD   probe_offset_get_next_cmd
#define PIO_SWD_OFFSET_READ_CMD       probe_offset_read_cmd

#else
/*
 * Pre-assembled PIO instructions (fallback when pioasm not available)
 *
 * .program probe
 * .side_set 1 opt
 *
 * public write_cmd:
 * public turnaround_cmd:
 *     pull
 * write_bitloop:
 *     out pins, 1             [1]  side 0x0
 *     jmp x-- write_bitloop   [1]  side 0x1
 *
 * .wrap_target
 * public get_next_cmd:
 *     pull                         side 0x0
 *     out x, 8
 *     out pindirs, 1
 *     out pc, 5
 *
 * read_bitloop:
 *     nop
 * public read_cmd:
 *     in pins, 1              [1]  side 0x1
 *     jmp x-- read_bitloop         side 0x0
 *     push
 * .wrap
 */

static const uint16_t pio_swd_program_instructions[] = {
    /* 0: pull                       - write_cmd, turnaround_cmd */
    0x80a0,
    /* 1: out pins, 1 [1] side 0x0   - write_bitloop */
    0x6101,
    /* 2: jmp x-- 1 [1] side 0x1     - continue write loop */
    0x1541,
    /* 3: pull side 0x0              - get_next_cmd (wrap_target) */
    0x90a0,
    /* 4: out x, 8                   - get bit count */
    0x6028,
    /* 5: out pindirs, 1             - set SWDIO direction */
    0x6081,
    /* 6: out pc, 5                  - jump to command routine */
    0x60a5,
    /* 7: nop                        - read_bitloop (mov y, y) */
    0xa042,
    /* 8: in pins, 1 [1] side 0x1    - read_cmd */
    0x5501,
    /* 9: jmp x-- 7 side 0x0         - continue read loop */
    0x1047,
    /* 10: push                      */
    0x8020,
};

static const pio_program_t pio_swd_program = {
    .instructions = pio_swd_program_instructions,
    .length = ARRAY_SIZE(pio_swd_program_instructions),
    .origin = -1,
    .pio_version = 0,
};

#define PIO_SWD_PROGRAM_LENGTH  ARRAY_SIZE(pio_swd_program_instructions)
#define PIO_SWD_WRAP_TARGET     3   /* get_next_cmd */
#define PIO_SWD_WRAP            10  /* after push */

/* Entry points (offsets) */
#define PIO_SWD_OFFSET_WRITE_CMD      0
#define PIO_SWD_OFFSET_TURNAROUND_CMD 0
#define PIO_SWD_OFFSET_GET_NEXT_CMD   3
#define PIO_SWD_OFFSET_READ_CMD       8

#endif /* PIO_GENERATED_HEADERS */

/* Command encoding for PIO FIFO */
#define PIO_SWD_CMD_WRITE   PIO_SWD_OFFSET_WRITE_CMD
#define PIO_SWD_CMD_READ    PIO_SWD_OFFSET_READ_CMD

/* OE control availability - uses explicit GPIO control (not PIO side-set)
 * because OE pin may not be adjacent to SWCLK */
#if defined(PROBE_IO_OEN) && (PROBE_IO_OEN >= 0)
#define PIO_SWD_HAS_OE_CONTROL  1
#else
#define PIO_SWD_HAS_OE_CONTROL  0
#endif

/*
 * Format a command word for the PIO state machine
 *
 * The command format is:
 * - Bits 13:9 - Command address (entry point offset)
 * - Bit 8     - Output enable (1=output, 0=input)
 * - Bits 7:0  - Bit count minus 1
 *
 * @param bit_count Number of bits to transfer
 * @param output    true for write/output, false for read/input
 * @param cmd       Command offset (PIO_SWD_CMD_WRITE or PIO_SWD_CMD_READ)
 * @return          Formatted command word
 */
static inline uint32_t pio_swd_format_cmd(uint32_t bit_count, bool output, uint32_t cmd)
{
    return ((cmd & 0x1f) << 9) | ((output ? 1 : 0) << 8) | ((bit_count - 1) & 0xff);
}

/**
 * @brief Initialize PIO-based SWD
 *
 * Configures a PIO state machine for high-speed SWD operations.
 *
 * @return 0 on success, negative error code otherwise
 */
int pio_swd_init(void);

/**
 * @brief Deinitialize PIO SWD
 */
void pio_swd_deinit(void);

/**
 * @brief Check if PIO SWD is available
 *
 * @return true if PIO SWD is initialized and available
 */
bool pio_swd_available(void);

/**
 * @brief Set SWD clock frequency for PIO
 *
 * @param freq_hz Desired clock frequency in Hz
 */
void pio_swd_set_freq(uint32_t freq_hz);

/**
 * @brief Write bits via PIO SWD
 *
 * @param bit_count Number of bits to write (1-32)
 * @param data      Data to write (LSB first)
 */
void pio_swd_write_bits(uint32_t bit_count, uint32_t data);

/**
 * @brief Read bits via PIO SWD
 *
 * @param bit_count Number of bits to read (1-32)
 * @return          Read data (LSB first)
 */
uint32_t pio_swd_read_bits(uint32_t bit_count);

/**
 * @brief Perform turnaround (switch bus direction)
 *
 * @param to_output true if switching to output mode, false for input
 */
void pio_swd_turnaround(bool to_output);

/**
 * @brief Switch to read mode (SWDIO as input)
 *
 * Waits for PIO idle before allowing mode change.
 */
void pio_swd_read_mode(void);

/**
 * @brief Switch to write mode (SWDIO as output)
 *
 * Waits for PIO idle before allowing mode change.
 */
void pio_swd_write_mode(void);

/**
 * @brief Output high-impedance clocks
 *
 * Clock SWCLK with SWDIO tristated, used for line turnaround.
 *
 * @param count Number of clock cycles
 */
void pio_swd_hiz_clocks(uint32_t count);

/**
 * @brief Execute SWJ sequence via PIO
 *
 * @param count Number of bits to output
 * @param data  Data to output (LSB first)
 */
void pio_swd_sequence(uint32_t count, const uint8_t *data);

/**
 * @brief Perform complete SWD transfer via PIO
 *
 * This is the high-level transfer function that handles the complete
 * SWD protocol including request, ACK, and data phases.
 *
 * @param request Request byte (APnDP, RnW, A[2:3])
 * @param data    Pointer to data (write) or result (read)
 * @return        ACK value (OK=1, WAIT=2, FAULT=4) or error
 */
uint8_t pio_swd_transfer(uint32_t request, uint32_t *data);

/**
 * @brief Check if OE (Output Enable) control is available
 *
 * OE control is used for level shifter direction control on
 * Debug Probe hardware with external level shifters.
 *
 * @return true if OE control is available and initialized
 */
static inline bool pio_swd_has_oe_control(void)
{
#if PIO_SWD_HAS_OE_CONTROL
    return true;
#else
    return false;
#endif
}

#endif /* CONFIG_SOC_RP2040 */
