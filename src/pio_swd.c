/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - PIO-based SWD Implementation
 *
 * High-speed SWD implementation using RP2040 PIO state machines.
 * Port of probe.c PIO functions from the original debugprobe.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_SOC_RP2040

#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "probe_config.h"
#include "probe.h"
#include "pio_swd.h"
#include "dap.h"

LOG_MODULE_REGISTER(pio_swd, CONFIG_LOG_DEFAULT_LEVEL);

/* PIO instance and state machine */
#define PIO_SWD_PIO     pio0
#define PIO_SWD_SM      0

/* PIO_FDEBUG_TXSTALL_LSB is defined in hardware/regs/pio.h */

/* State tracking */
static bool pio_swd_initialized = false;
static uint32_t pio_program_offset = 0;

/*
 * Output Enable (OE) Control for Level Shifters
 *
 * The OE pin controls the level shifter direction:
 *   - OE HIGH: Probe drives the line (output mode)
 *   - OE LOW:  Probe releases the line (input mode / high-Z)
 *
 * Since OE pin may not be adjacent to SWCLK, we use explicit GPIO
 * control instead of PIO side-set.
 */
#if PIO_SWD_HAS_OE_CONTROL

static inline void pio_swd_oe_output(void)
{
    gpio_put(PROBE_IO_OEN, 1);  /* Enable output driver */
}

static inline void pio_swd_oe_input(void)
{
    gpio_put(PROBE_IO_OEN, 0);  /* Tri-state / input mode */
}

static void pio_swd_oe_init(void)
{
    gpio_init(PROBE_IO_OEN);
    gpio_set_dir(PROBE_IO_OEN, GPIO_OUT);
    gpio_put(PROBE_IO_OEN, 0);  /* Start in input mode (safe state) */
    LOG_INF("OE control initialized on GPIO %d", PROBE_IO_OEN);
}

#else

/* No-op macros when OE control is not available */
#define pio_swd_oe_output()  ((void)0)
#define pio_swd_oe_input()   ((void)0)
#define pio_swd_oe_init()    ((void)0)

#endif /* PIO_SWD_HAS_OE_CONTROL */

/*
 * Wait for PIO state machine to become idle
 *
 * This uses the FDEBUG TXSTALL flag to detect when the state machine
 * has completed all pending operations and is waiting for more data.
 * Critical for safe mode transitions (read/write direction changes).
 */
static void probe_wait_idle(void)
{
    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;
    uint32_t stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);

    /* Clear the TXSTALL flag by writing 1 */
    pio->fdebug = stall_mask;

    /* Wait until TXSTALL is set (SM is stalled waiting for TX FIFO) */
    while (!(pio->fdebug & stall_mask)) {
        /* Busy wait - this is typically very short */
    }
}

/*
 * Switch to read mode (SWDIO as input)
 */
void pio_swd_read_mode(void)
{
    probe_wait_idle();
    /* Direction will be set by the PIO program via the command format */
}

/*
 * Switch to write mode (SWDIO as output)
 */
void pio_swd_write_mode(void)
{
    probe_wait_idle();
    /* Direction will be set by the PIO program via the command format */
}

/*
 * Output high-impedance clocks (for line turnaround)
 * Clock SWCLK with SWDIO tristated
 */
void pio_swd_hiz_clocks(uint32_t count)
{
    probe_wait_idle();
    /* Use a read command with SWDIO as input to tristate the line */
    uint32_t cmd = pio_swd_format_cmd(count, false, PIO_SWD_CMD_READ);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
    /* Discard the read data */
    pio_sm_get_blocking(PIO_SWD_PIO, PIO_SWD_SM);
}

/*
 * Initialize PIO state machine for SWD
 *
 * Uses 1-bit side-set for SWCLK. OE control (if available) is handled
 * separately via explicit GPIO control since OE pin may not be adjacent.
 */
static void pio_swd_sm_init(uint32_t freq_hz)
{
    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Calculate clock divider for desired frequency
     * PIO runs at system clock, SWD needs 4 cycles per bit
     */
    uint32_t sys_freq = clock_get_hz(clk_sys);
    float clkdiv = (float)sys_freq / (4.0f * freq_hz);

    /* Configure state machine */
    pio_sm_config c = pio_get_default_sm_config();

    /* Set up side-set (1 bit for SWCLK, optional) */
    sm_config_set_sideset(&c, 2, true, false);  /* 2 bits total (1 side-set + opt), not pindirs */

    /* Set SWCLK as side-set pin */
    sm_config_set_sideset_pins(&c, PROBE_SWCLK_PIN);

    /* OUT shifts right, autopull at 32 bits */
    sm_config_set_out_shift(&c, true, true, 32);

    /* IN shifts right, autopush at 32 bits */
    sm_config_set_in_shift(&c, true, true, 32);

    /* Set SWDIO as OUT and IN pin */
    sm_config_set_out_pins(&c, PROBE_SWDIO_PIN, 1);
    sm_config_set_in_pins(&c, PROBE_SWDIO_PIN);
    sm_config_set_set_pins(&c, PROBE_SWDIO_PIN, 1);

    /* Set clock divider */
    sm_config_set_clkdiv(&c, clkdiv);

    /* Initialize GPIO for PIO use */
    pio_gpio_init(pio, PROBE_SWCLK_PIN);
    pio_gpio_init(pio, PROBE_SWDIO_PIN);

    /* Set SWCLK as output, SWDIO as output initially */
    pio_sm_set_consecutive_pindirs(pio, sm, PROBE_SWCLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PROBE_SWDIO_PIN, 1, true);

    /* Initialize and enable state machine */
    pio_sm_init(pio, sm, pio_program_offset + PIO_SWD_OFFSET_GET_NEXT_CMD, &c);
    pio_sm_set_enabled(pio, sm, true);
}

/*
 * Initialize PIO-based SWD
 */
int pio_swd_init(void)
{
    PIO pio = PIO_SWD_PIO;

    if (pio_swd_initialized) {
        return 0;
    }

    /* Check if we can claim a state machine */
    if (!pio_can_add_program(pio, &pio_swd_program)) {
        LOG_ERR("Cannot add PIO program - no space");
        return -ENOMEM;
    }

    /* Add program to PIO */
    pio_program_offset = pio_add_program(pio, &pio_swd_program);

    /* Initialize OE control if available */
    pio_swd_oe_init();

    /* Initialize state machine with default frequency */
    pio_swd_sm_init(DAP_DEFAULT_SWJ_CLOCK);

    pio_swd_initialized = true;
    LOG_INF("PIO SWD initialized at offset %u", pio_program_offset);

    return 0;
}

/*
 * Deinitialize PIO SWD
 */
void pio_swd_deinit(void)
{
    if (!pio_swd_initialized) {
        return;
    }

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Disable state machine */
    pio_sm_set_enabled(pio, sm, false);

    /* Remove program */
    pio_remove_program(pio, &pio_swd_program, pio_program_offset);

    /* Reset GPIO to regular mode */
    gpio_set_function(PROBE_SWCLK_PIN, GPIO_FUNC_SIO);
    gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);

#if PIO_SWD_HAS_OE_CONTROL
    /* Reset OE pin and set low (disable output / safe state) */
    gpio_put(PROBE_IO_OEN, 0);
#endif

    pio_swd_initialized = false;
    LOG_INF("PIO SWD deinitialized");
}

/*
 * Check if PIO SWD is available
 */
bool pio_swd_available(void)
{
    return pio_swd_initialized;
}

/*
 * Set SWD clock frequency
 */
void pio_swd_set_freq(uint32_t freq_hz)
{
    if (!pio_swd_initialized) {
        return;
    }

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Calculate new clock divider */
    uint32_t sys_freq = clock_get_hz(clk_sys);
    float clkdiv = (float)sys_freq / (4.0f * freq_hz);

    /* Update clock divider */
    pio_sm_set_clkdiv(pio, sm, clkdiv);

    LOG_DBG("PIO SWD frequency set to %u Hz", freq_hz);
}

/*
 * Write bits via PIO
 */
void pio_swd_write_bits(uint32_t bit_count, uint32_t data)
{
    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Enable output driver for level shifter */
    pio_swd_oe_output();

    /* Format command: write, output enable, bit_count */
    uint32_t cmd = pio_swd_format_cmd(bit_count, true, PIO_SWD_CMD_WRITE);

    /* Send command and data */
    pio_sm_put_blocking(pio, sm, cmd);
    pio_sm_put_blocking(pio, sm, data);
}

/*
 * Read bits via PIO
 */
uint32_t pio_swd_read_bits(uint32_t bit_count)
{
    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Disable output driver for level shifter (input mode) */
    pio_swd_oe_input();

    /* Format command: read, input mode, bit_count */
    uint32_t cmd = pio_swd_format_cmd(bit_count, false, PIO_SWD_CMD_READ);

    /* Send command */
    pio_sm_put_blocking(pio, sm, cmd);

    /* Get result and align to LSB */
    uint32_t data = pio_sm_get_blocking(pio, sm);
    if (bit_count < 32) {
        data >>= (32 - bit_count);
    }

    return data;
}

/*
 * Perform turnaround
 */
void pio_swd_turnaround(bool to_output)
{
    /* Wait for PIO idle before changing direction */
    probe_wait_idle();

    /* Set OE based on new direction */
    if (to_output) {
        pio_swd_oe_output();
    } else {
        pio_swd_oe_input();
    }

    /* Write 1 bit with appropriate direction */
    uint32_t cmd = pio_swd_format_cmd(1, to_output, PIO_SWD_OFFSET_TURNAROUND_CMD);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, 0);
}

/*
 * Execute SWJ sequence
 */
void pio_swd_sequence(uint32_t count, const uint8_t *data)
{
    uint32_t val;
    uint32_t n;

    /* SWJ sequences are always output */
    pio_swd_oe_output();

    while (count > 0) {
        val = *data++;
        n = (count > 8) ? 8 : count;
        count -= n;
        pio_swd_write_bits(n, val);
    }
}

/*
 * Compute parity of a 32-bit value
 */
static inline uint32_t parity32(uint32_t val)
{
    val ^= val >> 16;
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return val & 1;
}

/*
 * Perform complete SWD transfer
 *
 * Implements the full SWD protocol:
 * 1. Request phase (8 bits: start, APnDP, RnW, A[2:3], parity, stop, park)
 * 2. Turnaround
 * 3. ACK phase (3 bits)
 * 4. Data phase (32 bits + parity) - read or write
 * 5. Turnaround (for reads)
 */
uint8_t pio_swd_transfer(uint32_t request, uint32_t *data)
{
    uint32_t req;
    uint32_t ack;
    uint32_t val;
    uint32_t parity;

    /* Build request packet:
     * Bit 0: Start (always 1)
     * Bit 1: APnDP
     * Bit 2: RnW
     * Bits 3-4: A[2:3]
     * Bit 5: Parity
     * Bit 6: Stop (always 0)
     * Bit 7: Park (always 1)
     */
    req = 0x81;  /* Start=1, Stop=0, Park=1 */
    req |= (request & 0x0f) << 1;  /* APnDP, RnW, A[2:3] */

    /* Calculate and add parity */
    parity = parity32(request & 0x0f);
    req |= parity << 5;

    /* Send request (8 bits) */
    pio_swd_write_bits(8, req);

    /* Turnaround to input */
    pio_swd_turnaround(false);

    /* Read ACK (3 bits) */
    ack = pio_swd_read_bits(3);

    if (ack == DAP_TRANSFER_OK) {
        /* Data phase */
        if (request & DAP_TRANSFER_RnW) {
            /* Read operation */
            val = pio_swd_read_bits(32);
            parity = pio_swd_read_bits(1);

            /* Turnaround to output */
            pio_swd_turnaround(true);

            /* Verify parity */
            if (parity != parity32(val)) {
                return DAP_TRANSFER_ERROR;
            }

            if (data) {
                *data = val;
            }
        } else {
            /* Write operation - turnaround to output */
            pio_swd_turnaround(true);

            /* Write data and parity */
            val = data ? *data : 0;
            pio_swd_write_bits(32, val);
            pio_swd_write_bits(1, parity32(val));
        }

        /* Idle cycles (configured via DAP_TransferConfigure) */
        uint8_t idle = dap_get_idle_cycles();
        if (idle > 0) {
            pio_swd_write_bits(idle, 0);
        }
    } else if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT) {
        /* Protocol error handling */
        if (request & DAP_TRANSFER_RnW) {
            /* Dummy read data phase */
            pio_swd_read_bits(33);
        }
        pio_swd_turnaround(true);
        if (!(request & DAP_TRANSFER_RnW)) {
            /* Dummy write data phase */
            pio_swd_write_bits(33, 0);
        }
    } else {
        /* Protocol error - no valid ACK */
        pio_swd_turnaround(true);
    }

    return (uint8_t)ack;
}

/*
 * Shell commands for PIO debugging
 */
#include <zephyr/shell/shell.h>

/* Helper to print one PIO block status */
static void print_pio_block_status(const struct shell *sh, PIO pio, int pio_num,
                                   int prog_offset, int prog_len)
{
    shell_print(sh, "PIO%d:", pio_num);

    /* Show instruction memory usage if known */
    if (prog_len > 0) {
        shell_print(sh, "  Instructions: %d/32 (offset %d-%d)",
                    prog_len, prog_offset, prog_offset + prog_len - 1);
    }

    /* Show which SMs are enabled */
    int enabled_count = 0;
    for (int sm = 0; sm < 4; sm++) {
        bool enabled = (pio->ctrl & (1u << sm)) != 0;
        if (enabled) {
            enabled_count++;
            uint32_t flevel = pio->flevel;
            uint tx_level = (flevel >> (sm * 8)) & 0xF;
            uint rx_level = (flevel >> (sm * 8 + 4)) & 0xF;
            uint32_t pc = pio_sm_get_pc(pio, sm);

            shell_print(sh, "  SM%d: enabled, PC=%u, TX=%u/4, RX=%u/4",
                        sm, pc, tx_level, rx_level);
        }
    }
    if (enabled_count == 0) {
        shell_print(sh, "  No state machines enabled");
    }

    /* Show FDEBUG flags if any are set */
    uint32_t fdebug = pio->fdebug;
    if (fdebug) {
        shell_print(sh, "  FDEBUG: 0x%08x", fdebug);
    }
}

/* pio all - show status of both PIO blocks */
static int cmd_pio_all(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "RP2040 PIO Overview:");

    /* PIO0 has the SWD program loaded */
    if (pio_swd_initialized) {
        print_pio_block_status(sh, pio0, 0, pio_program_offset,
                               pio_swd_program.length);
    } else {
        print_pio_block_status(sh, pio0, 0, 0, 0);
    }

    /* PIO1 is unused */
    print_pio_block_status(sh, pio1, 1, 0, 0);

    return 0;
}

/* pio status - show PIO SWD state machine status */
static int cmd_pio_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    shell_print(sh, "PIO SWD Status (PIO0 SM0):");
    shell_print(sh, "  Initialized: %s", pio_swd_initialized ? "yes" : "no");

    if (!pio_swd_initialized) {
        return 0;
    }

    shell_print(sh, "  Program offset: %u", pio_program_offset);

    /* Check if SM is enabled */
    bool sm_enabled = (pio->ctrl & (1u << sm)) != 0;
    shell_print(sh, "  SM enabled: %s", sm_enabled ? "yes" : "no");

    /* Show FIFO levels */
    uint32_t flevel = pio->flevel;
    uint tx_level = (flevel >> (sm * 8)) & 0xF;
    uint rx_level = (flevel >> (sm * 8 + 4)) & 0xF;
    shell_print(sh, "  TX FIFO level: %u/4", tx_level);
    shell_print(sh, "  RX FIFO level: %u/4", rx_level);

    /* Show current instruction address */
    uint32_t pc = pio_sm_get_pc(pio, sm);
    shell_print(sh, "  Program counter: %u", pc - pio_program_offset);

#if PIO_SWD_HAS_OE_CONTROL
    shell_print(sh, "  OE pin (GPIO%d): %s", PROBE_IO_OEN,
                gpio_get(PROBE_IO_OEN) ? "output" : "input/hi-z");
#endif

    return 0;
}

/* pio debug - show PIO debug registers */
static int cmd_pio_debug(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    shell_print(sh, "PIO Debug Info:");

    if (!pio_swd_initialized) {
        shell_print(sh, "  PIO SWD not initialized");
        return 0;
    }

    /* Read FDEBUG register */
    uint32_t fdebug = pio->fdebug;
    uint32_t stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    uint32_t over_mask = 1u << (PIO_FDEBUG_TXOVER_LSB + sm);
    uint32_t under_mask = 1u << (PIO_FDEBUG_RXUNDER_LSB + sm);
    uint32_t rxstall_mask = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm);

    shell_print(sh, "  FDEBUG register: 0x%08x", fdebug);
    shell_print(sh, "  TX stall: %s", (fdebug & stall_mask) ? "yes" : "no");
    shell_print(sh, "  TX overflow: %s", (fdebug & over_mask) ? "yes" : "no");
    shell_print(sh, "  RX underflow: %s", (fdebug & under_mask) ? "yes" : "no");
    shell_print(sh, "  RX stall: %s", (fdebug & rxstall_mask) ? "yes" : "no");

    /* Show clock divider */
    uint32_t clkdiv = pio->sm[sm].clkdiv;
    uint32_t int_div = clkdiv >> 16;
    uint32_t frac_div = (clkdiv >> 8) & 0xFF;
    shell_print(sh, "  Clock divider: %u.%02u", int_div, (frac_div * 100) / 256);

    /* Calculate effective frequency */
    uint32_t sys_freq = clock_get_hz(clk_sys);
    float eff_div = (float)int_div + (float)frac_div / 256.0f;
    if (eff_div > 0) {
        uint32_t pio_freq = (uint32_t)(sys_freq / eff_div / 4.0f);
        shell_print(sh, "  Effective SWCLK: ~%u Hz", pio_freq);
    }

    return 0;
}

/* pio reset - clear PIO debug flags */
static int cmd_pio_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!pio_swd_initialized) {
        shell_print(sh, "PIO SWD not initialized");
        return 0;
    }

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;

    /* Clear all debug flags for this SM */
    uint32_t clear_mask = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)) |
                          (1u << (PIO_FDEBUG_TXOVER_LSB + sm)) |
                          (1u << (PIO_FDEBUG_RXUNDER_LSB + sm)) |
                          (1u << (PIO_FDEBUG_RXSTALL_LSB + sm));
    pio->fdebug = clear_mask;

    shell_print(sh, "PIO debug flags cleared");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pio,
    SHELL_CMD(all, NULL, "Show all PIO blocks overview", cmd_pio_all),
    SHELL_CMD(status, NULL, "Show PIO SWD state machine status", cmd_pio_status),
    SHELL_CMD(debug, NULL, "Show PIO SWD debug registers", cmd_pio_debug),
    SHELL_CMD(reset, NULL, "Clear PIO SWD debug flags", cmd_pio_reset),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pio, &sub_pio, "PIO diagnostics (RP2040)", NULL);

#endif /* CONFIG_SOC_RP2040 */
