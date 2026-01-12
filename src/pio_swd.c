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
#include "led.h"

LOG_MODULE_REGISTER(pio_swd, CONFIG_LOG_DEFAULT_LEVEL);

/* PIO instance and state machine */
#define PIO_SWD_PIO     pio0
#define PIO_SWD_SM      0

/* PIO_FDEBUG_TXSTALL_LSB is defined in hardware/regs/pio.h */

/* State tracking */
static bool pio_swd_initialized = false;
static uint32_t pio_program_offset = 0;

/* Failure tracking for wiring error detection */
#define SWD_FAILURE_THRESHOLD 3   /* Consecutive failures before error indication */
static uint32_t swd_consecutive_failures = 0;
static bool swd_error_indicated = false;

/*
 * Format command with program offset applied
 * The cmd offset needs to be adjusted by program_offset since
 * the PIO program may be loaded at a non-zero address
 */
static inline uint32_t pio_cmd_with_offset(uint32_t bit_count, bool output, uint32_t cmd_offset)
{
    uint32_t abs_addr = (pio_program_offset + cmd_offset) & 0x1f;
    return (abs_addr << 9) | ((output ? 1 : 0) << 8) | ((bit_count - 1) & 0xff);
}

/*
 * Level Shifter Control for Debug Probe Hardware
 *
 * With probe_oen.pio, SWDIOEN is controlled via PIO side-set, synchronized
 * with clock edges. The side-set automatically handles:
 *   - Write: SWDIOEN=0 (enabled) with clock toggling
 *   - Read/turnaround: SWDIOEN=1 (disabled) with clock toggling
 *
 * These functions are now NO-OPs since SWDIOEN is PIO-controlled.
 * They're kept for API compatibility with code that calls them.
 */
#if PIO_SWD_HAS_OE_CONTROL

static inline void pio_swd_oe_output(void)
{
    /* No-op: SWDIOEN is controlled by PIO side-set during write commands */
}

static inline void pio_swd_oe_input(void)
{
    /* No-op: SWDIOEN is controlled by PIO side-set during read commands */
}

static void pio_swd_oe_init(void)
{
    /* SWDIOEN is initialized in pio_swd_sm_init() as a PIO-controlled pin */
    LOG_INF("SWDIOEN on GPIO %d controlled via PIO side-set", PROBE_IO_OEN);
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
 *
 * Like original debugprobe's probe_read_mode() - sends a CMD_SKIP command
 * to explicitly set pindirs to input, then waits for completion.
 * This gives the level shifter time to switch direction.
 */
void pio_swd_read_mode(void)
{
    /* Send skip command with dir=input to set pindirs without transfer.
     * CMD_SKIP jumps to get_next_cmd which sets pindirs and then waits.
     * bit_count=0 is special - no actual bits transferred.
     */
    uint32_t cmd = pio_cmd_with_offset(0, false, PIO_SWD_OFFSET_GET_NEXT_CMD);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
    probe_wait_idle();
}

/*
 * Switch to write mode (SWDIO as output)
 *
 * Like original debugprobe's probe_write_mode() - sends a CMD_SKIP command
 * to explicitly set pindirs to output, then waits for completion.
 */
void pio_swd_write_mode(void)
{
    /* Send skip command with dir=output to set pindirs without transfer */
    uint32_t cmd = pio_cmd_with_offset(0, true, PIO_SWD_OFFSET_GET_NEXT_CMD);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
    probe_wait_idle();
}

/*
 * Output high-impedance clocks (for line turnaround)
 * Clock SWCLK with SWDIO tristated
 *
 * Uses the write/turnaround command with output=false to tristate SWDIO
 * while generating clock pulses. This is the same approach as the original
 * debugprobe's probe_hiz_clocks() function.
 */
void pio_swd_hiz_clocks(uint32_t count)
{
    probe_wait_idle();
    /* Use turnaround command with output=false to tristate SWDIO while clocking */
    uint32_t cmd = pio_cmd_with_offset(count, false, PIO_SWD_OFFSET_TURNAROUND_CMD);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
    pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, 0);  /* Data (ignored, pin is tristated) */
}

/*
 * Initialize PIO state machine for SWD
 *
 * Uses probe.pio program with 1-bit side-set for SWCLK control.
 * Pin layout matching original board_debug_probe_config.h:
 *   - GPIO12: SWCLK (side-set controlled)
 *   - GPIO13: SWDI  (data input from target, separate pin)
 *   - GPIO14: SWDIO (data output to target)
 *
 * The level shifter has auto-direction sensing - no OE control needed.
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

    /* Set up 1-bit optional side-set for SWCLK only
     * Total side-set bits = 2 (1 enable + 1 value)
     * Remaining delay bits = 3
     */
    sm_config_set_sideset(&c, 2, true, false);  /* 2 bits total, optional, no pindirs */

    /* Set wrap points for the program */
    sm_config_set_wrap(&c, pio_program_offset + PIO_SWD_WRAP_TARGET,
                       pio_program_offset + PIO_SWD_WRAP);

    /* Side-set pin is SWCLK (GPIO12) */
    sm_config_set_sideset_pins(&c, PROBE_SWCLK_PIN);

    /* OUT shifts right, autopull OFF, threshold 0 (matches original) */
    sm_config_set_out_shift(&c, true, false, 0);

    /* IN shifts right, autopush OFF, threshold 0 (matches original) */
    sm_config_set_in_shift(&c, true, false, 0);

    /* Set SWDIO (GPIO14) as OUT pin */
    sm_config_set_out_pins(&c, PROBE_SWDIO_PIN, 1);
    sm_config_set_set_pins(&c, PROBE_SWDIO_PIN, 1);

    /* Set SWDI (GPIO13) as IN pin */
#if defined(PROBE_SWDI_PIN) && (PROBE_SWDI_PIN >= 0)
    sm_config_set_in_pins(&c, PROBE_SWDI_PIN);
#else
    sm_config_set_in_pins(&c, PROBE_SWDIO_PIN);
#endif

    /* Set clock divider */
    sm_config_set_clkdiv(&c, clkdiv);

    /* Initialize GPIO for PIO use - matching original probe_gpio_init() */
    pio_gpio_init(pio, PROBE_SWCLK_PIN); /* SWCLK - side-set controlled */
    pio_gpio_init(pio, PROBE_SWDIO_PIN); /* SWDIO - out controlled */

    /* Initialize SWDI (GPIO13) as input for reading from target
     * The Debug Probe has separate input/output paths through the level shifter:
     *   - SWDIO (GPIO14) outputs to target
     *   - SWDI (GPIO13) receives from target
     * We don't use pio_gpio_init because SWDI is read-only and doesn't need PIO funcsel.
     * The PIO 'in pins' instruction reads pad values regardless of funcsel.
     */
#if defined(PROBE_SWDI_PIN) && (PROBE_SWDI_PIN >= 0)
    gpio_init(PROBE_SWDI_PIN);
    gpio_set_dir(PROBE_SWDI_PIN, GPIO_IN);
    /* Enable pull-up on SWDI to see target's idle state */
    gpio_pull_up(PROBE_SWDI_PIN);
#endif

    /* Pull-up on SWDIO - idle state is HIGH */
    gpio_pull_up(PROBE_SWDIO_PIN);

    /* Set SWCLK and SWDIO as outputs initially (2 pins from PROBE_PIN_OFFSET)
     * Note: This sets GPIO12,13 but GPIO13 (SWDI) is input - that's OK
     * because IN samples from pad regardless of pindir.
     * The `out pindirs` instruction controls SWDIO (GPIO14) dynamically.
     */
    pio_sm_set_consecutive_pindirs(pio, sm, PROBE_PIN_OFFSET, 2, true);

    /* Initialize and enable state machine at get_next_cmd */
    pio_sm_init(pio, sm, pio_program_offset + PIO_SWD_OFFSET_GET_NEXT_CMD, &c);
    pio_sm_set_enabled(pio, sm, true);

    LOG_INF("PIO SM initialized: SWCLK=%d, SWDI=%d, SWDIO=%d",
            PROBE_SWCLK_PIN, PROBE_SWDI_PIN, PROBE_SWDIO_PIN);
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

    /* Reset GPIO to regular mode - matching original probe_gpio_deinit() */
    gpio_deinit(PROBE_SWCLK_PIN);
    gpio_disable_pulls(PROBE_SWCLK_PIN);
    gpio_deinit(PROBE_SWDIO_PIN);
    gpio_disable_pulls(PROBE_SWDIO_PIN);

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
    uint32_t cmd = pio_cmd_with_offset(bit_count, true, PIO_SWD_CMD_WRITE);

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
    uint32_t cmd = pio_cmd_with_offset(bit_count, false, PIO_SWD_CMD_READ);

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
 *
 * Critical timing: The turnaround is when bus ownership switches between
 * host and target. We must coordinate:
 * 1. PIO GPIO direction (via command bit 8)
 * 2. Level shifter direction (via SWDIR GPIO)
 *
 * For switching to INPUT: Change SWDIR first so we can receive target's signal
 * For switching to OUTPUT: Change SWDIR after so there's no conflict
 */
void pio_swd_turnaround(bool to_output)
{
    /* Wait for PIO idle before changing direction */
    probe_wait_idle();

    if (to_output) {
        /* Switching TO output: do turnaround first, then change level shifter */
        uint32_t cmd = pio_cmd_with_offset(1, true, PIO_SWD_OFFSET_TURNAROUND_CMD);
        pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
        pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, 0);
        probe_wait_idle();
        pio_swd_oe_output();
    } else {
        /* Switching TO input: change level shifter FIRST so we can read */
        pio_swd_oe_input();
        uint32_t cmd = pio_cmd_with_offset(1, false, PIO_SWD_OFFSET_TURNAROUND_CMD);
        pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, cmd);
        pio_sm_put_blocking(PIO_SWD_PIO, PIO_SWD_SM, 0);
    }
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
 * Track transfer result for wiring error detection
 */
static inline void track_transfer_result(uint8_t ack)
{
    if (ack == DAP_TRANSFER_OK) {
        /* Success - reset failure counter */
        if (swd_consecutive_failures > 0) {
            LOG_DBG("SWD: success after %u failures", swd_consecutive_failures);
            swd_consecutive_failures = 0;
            if (swd_error_indicated) {
                swd_error_indicated = false;
                led_swd_error_clear();
            }
        }
    } else if (ack != DAP_TRANSFER_WAIT) {
        /* Failure (but not WAIT which is normal during busy) */
        swd_consecutive_failures++;

        if (swd_consecutive_failures >= SWD_FAILURE_THRESHOLD) {
            if (!swd_error_indicated) {
                LOG_WRN("SWD: %u consecutive failures - possible wiring issue",
                        swd_consecutive_failures);
                swd_error_indicated = true;
            }
            /* Call on every failure to extend the blink timeout */
            led_swd_error();
        }
    }
}

/*
 * Perform complete SWD transfer
 *
 * Implements the full SWD protocol matching original debugprobe:
 * 1. Request phase (8 bits: start, APnDP, RnW, A[2:3], parity, stop, park)
 * 2. Read turnaround + ACK together (4 bits, discard first)
 * 3. Data phase (32 bits + parity) - read or write
 * 4. Turnaround (for reads)
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

    /* Read turnaround + ACK together (like original debugprobe)
     * NO explicit mode switch here - the command's out_en bit (set to false
     * by pio_swd_read_bits) tells the PIO program to set pindirs to input
     * via the `out pindirs, 1` instruction in get_next_cmd.
     *
     * turnaround is configurable (1-4 cycles), ACK is 3 bits
     * First N bits are turnaround (discarded), remaining 3 are ACK
     */
    uint8_t turnaround = dap_get_swd_turnaround();
    ack = pio_swd_read_bits(turnaround + 3);
    ack >>= turnaround;  /* Discard turnaround bits, keep 3 ACK bits */

    if (ack == DAP_TRANSFER_OK) {
        /* Data phase */
        if (request & DAP_TRANSFER_RnW) {
            /* Read operation - still in input mode from ACK read */
            val = pio_swd_read_bits(32);
            parity = pio_swd_read_bits(1);

            /* Verify parity */
            if (parity != parity32(val)) {
                ack = DAP_TRANSFER_ERROR;
            } else if (data) {
                *data = val;
            }

            /* Turnaround for line idle (like original: probe_hiz_clocks) */
            pio_swd_hiz_clocks(turnaround);
        } else {
            /* Write operation - need turnaround first */
            pio_swd_hiz_clocks(turnaround);

            /* Write data - NO explicit mode switch here.
             * pio_swd_write_bits sends out_en=true which tells PIO to set
             * pindirs to output via the `out pindirs, 1` instruction.
             */
            val = data ? *data : 0;
            pio_swd_write_bits(32, val);
            pio_swd_write_bits(1, parity32(val));
        }

        /* Idle cycles (configured via DAP_TransferConfigure) */
        uint8_t idle = dap_get_idle_cycles();
        if (idle > 0) {
            pio_swd_write_bits(idle, 0);
        }

        track_transfer_result(ack);
        return (uint8_t)ack;
    }

    if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT) {
        /* WAIT/FAULT handling - check data_phase setting */
        uint8_t data_phase = dap_get_swd_data_phase();
        if (data_phase && (request & DAP_TRANSFER_RnW)) {
            /* Dummy read data phase */
            pio_swd_read_bits(33);
        }
        pio_swd_hiz_clocks(turnaround);
        if (data_phase && !(request & DAP_TRANSFER_RnW)) {
            /* Dummy write data phase */
            pio_swd_write_bits(33, 0);
        }
        track_transfer_result(ack);
        return (uint8_t)ack;
    }

    /* Protocol error - no valid ACK, back off */
    pio_swd_read_bits(turnaround + 32 + 1);  /* Dummy read data phase */
    pio_swd_hiz_clocks(turnaround);

    track_transfer_result(ack);
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
    shell_print(sh, "  OE pin (GPIO%d): %s (active low)", PROBE_IO_OEN,
                gpio_get(PROBE_IO_OEN) ? "disabled" : "enabled");
#endif
#ifdef PROBE_SWDIR_PIN
    if (PROBE_SWDIR_PIN >= 0) {
        shell_print(sh, "  SWDIR pin (GPIO%d): %s (active low)", PROBE_SWDIR_PIN,
                    gpio_get(PROBE_SWDIR_PIN) ? "input" : "output");
    }
#endif

    /* Show failure tracking status */
    shell_print(sh, "  Consecutive failures: %u (threshold: %d)",
                swd_consecutive_failures, SWD_FAILURE_THRESHOLD);
    if (swd_error_indicated) {
        shell_print(sh, "  ERROR: Wiring problem detected!");
    }

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

/* pio test - test PIO state machine communication */
static int cmd_pio_test(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!pio_swd_initialized) {
        shell_print(sh, "PIO SWD not initialized");
        return -1;
    }

    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;
    uint32_t stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    int errors = 0;

    shell_print(sh, "PIO Test - exercising state machine...");

    /* Clear debug flags first */
    uint32_t clear_mask = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)) |
                          (1u << (PIO_FDEBUG_TXOVER_LSB + sm)) |
                          (1u << (PIO_FDEBUG_RXUNDER_LSB + sm)) |
                          (1u << (PIO_FDEBUG_RXSTALL_LSB + sm));
    pio->fdebug = clear_mask;

    /* Show initial state */
    uint32_t flevel = pio->flevel;
    uint tx_before = (flevel >> (sm * 8)) & 0xF;
    uint rx_before = (flevel >> (sm * 8 + 4)) & 0xF;
    uint32_t pc_before = pio_sm_get_pc(pio, sm);
    shell_print(sh, "  Before: PC=%u TX=%u RX=%u", pc_before, tx_before, rx_before);

    /* Test 1: Write operation (8 bits of 0xAA pattern) */
    shell_print(sh, "  Test 1: Write 8 bits (0xAA)...");
    uint32_t t0 = k_cycle_get_32();

    /* Clear stall, send command and data */
    pio->fdebug = stall_mask;
    uint32_t cmd = pio_cmd_with_offset(8, true, PIO_SWD_CMD_WRITE);
    pio_sm_put_blocking(pio, sm, cmd);
    pio_sm_put_blocking(pio, sm, 0xAA);

    /* Wait for completion (with timeout) */
    uint32_t timeout = 10000;
    while (!(pio->fdebug & stall_mask) && --timeout) {
        /* spin */
    }
    uint32_t t1 = k_cycle_get_32();

    if (timeout == 0) {
        shell_print(sh, "    FAIL: Write timed out!");
        errors++;
    } else {
        uint32_t cycles = t1 - t0;
        uint32_t us = (cycles * 1000000) / sys_clock_hw_cycles_per_sec();
        shell_print(sh, "    OK: completed in %u cycles (~%u us)", cycles, us);
    }

    /* Test 2: Read operation (8 bits) */
    shell_print(sh, "  Test 2: Read 8 bits...");
    t0 = k_cycle_get_32();

    /* Clear stall, send read command */
    pio->fdebug = stall_mask;
    cmd = pio_cmd_with_offset(8, false, PIO_SWD_CMD_READ);
    pio_sm_put_blocking(pio, sm, cmd);

    /* Wait for RX data (with timeout) */
    timeout = 10000;
    while (pio_sm_is_rx_fifo_empty(pio, sm) && --timeout) {
        /* spin */
    }
    t1 = k_cycle_get_32();

    if (timeout == 0) {
        shell_print(sh, "    FAIL: Read timed out!");
        errors++;
    } else {
        uint32_t data = pio_sm_get_blocking(pio, sm);
        data >>= 24;  /* Align 8 bits to LSB */
        uint32_t cycles = t1 - t0;
        uint32_t us = (cycles * 1000000) / sys_clock_hw_cycles_per_sec();
        shell_print(sh, "    OK: read 0x%02x in %u cycles (~%u us)", data, cycles, us);
    }

    /* Show final state */
    flevel = pio->flevel;
    uint tx_after = (flevel >> (sm * 8)) & 0xF;
    uint rx_after = (flevel >> (sm * 8 + 4)) & 0xF;
    uint32_t pc_after = pio_sm_get_pc(pio, sm);
    shell_print(sh, "  After: PC=%u TX=%u RX=%u", pc_after, tx_after, rx_after);

    /* Check debug flags */
    uint32_t fdebug = pio->fdebug;
    if (fdebug & (1u << (PIO_FDEBUG_TXOVER_LSB + sm))) {
        shell_print(sh, "  WARNING: TX overflow detected");
    }
    if (fdebug & (1u << (PIO_FDEBUG_RXUNDER_LSB + sm))) {
        shell_print(sh, "  WARNING: RX underflow detected");
    }
    if (fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + sm))) {
        shell_print(sh, "  WARNING: RX stall detected");
    }

    shell_print(sh, "  Result: %s", errors ? "FAILED" : "PASSED");
    return errors ? -1 : 0;
}

/* pio loopback - test SWD pin connectivity */
static int cmd_pio_loopback(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!pio_swd_initialized) {
        shell_print(sh, "PIO SWD not initialized");
        return -1;
    }

    shell_print(sh, "Debug Probe SWD Pin Test:");
    shell_print(sh, "  Hardware: SWCLK=%d, SWDI=%d, SWDIO=%d",
                PROBE_SWCLK_PIN, PROBE_SWDI_PIN, PROBE_SWDIO_PIN);
    shell_print(sh, "  (Target should be connected and powered)");

    /* Save PIO state and disable SM */
    PIO pio = PIO_SWD_PIO;
    uint sm = PIO_SWD_SM;
    bool was_enabled = (pio->ctrl & (1u << sm)) != 0;
    pio_sm_set_enabled(pio, sm, false);

    /* Test SWCLK (GPIO12) */
    shell_print(sh, "\n  1. SWCLK Test (GPIO%d):", PROBE_SWCLK_PIN);
    gpio_set_function(PROBE_SWCLK_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PROBE_SWCLK_PIN, GPIO_OUT);
    gpio_put(PROBE_SWCLK_PIN, 0);
    k_busy_wait(10);
    int clk_low = gpio_get(PROBE_SWCLK_PIN);
    gpio_put(PROBE_SWCLK_PIN, 1);
    k_busy_wait(10);
    int clk_high = gpio_get(PROBE_SWCLK_PIN);
    shell_print(sh, "    Drive 0->read %d, drive 1->read %d: %s",
                clk_low, clk_high,
                (clk_low == 0 && clk_high == 1) ? "OK" : "FAIL");

    /* Test SWDIO output (GPIO14) */
    shell_print(sh, "\n  2. SWDIO Output Test (GPIO%d):", PROBE_SWDIO_PIN);
    gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PROBE_SWDIO_PIN, GPIO_OUT);
    gpio_put(PROBE_SWDIO_PIN, 0);
    k_busy_wait(10);
    int dio_low = gpio_get(PROBE_SWDIO_PIN);
    gpio_put(PROBE_SWDIO_PIN, 1);
    k_busy_wait(10);
    int dio_high = gpio_get(PROBE_SWDIO_PIN);
    shell_print(sh, "    Drive 0->read %d, drive 1->read %d: %s",
                dio_low, dio_high,
                (dio_low == 0 && dio_high == 1) ? "OK" : "FAIL");

    /* Test SWDI input (GPIO13) - separate input from target */
    shell_print(sh, "\n  3. SWDI Input Test (GPIO%d):", PROBE_SWDI_PIN);
    gpio_init(PROBE_SWDI_PIN);
    gpio_set_dir(PROBE_SWDI_PIN, GPIO_IN);

    /* Read without pulls */
    gpio_disable_pulls(PROBE_SWDI_PIN);
    k_busy_wait(100);
    int swdi_no_pull = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "    Without pulls: %d", swdi_no_pull);

    /* Read with pull-up */
    gpio_pull_up(PROBE_SWDI_PIN);
    k_busy_wait(100);
    int swdi_pullup = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "    With pull-up: %d", swdi_pullup);

    /* Read with pull-down */
    gpio_disable_pulls(PROBE_SWDI_PIN);
    gpio_pull_down(PROBE_SWDI_PIN);
    k_busy_wait(100);
    int swdi_pulldown = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "    With pull-down: %d", swdi_pulldown);
    gpio_disable_pulls(PROBE_SWDI_PIN);

    if (swdi_pullup == 1 && swdi_pulldown == 0) {
        shell_print(sh, "    -> SWDI pin is floating (responds to pulls)");
        if (swdi_no_pull == 1) {
            shell_print(sh, "    -> Target may be providing pull-up on SWDIO line");
        }
    } else if (swdi_pullup == 1 && swdi_pulldown == 1) {
        shell_print(sh, "    -> SWDI held HIGH (target driving or strong pull-up)");
    } else if (swdi_pullup == 0 && swdi_pulldown == 0) {
        shell_print(sh, "    -> SWDI held LOW (check target connection)");
    }

    /* Check pad configurations */
    shell_print(sh, "\n  4. Pad Configuration:");
    for (int pin = PROBE_SWCLK_PIN; pin <= PROBE_SWDIO_PIN; pin++) {
        uint32_t *pad = (uint32_t *)(0x4001c000 + 4 + (pin * 4));
        uint32_t val = *pad;
        shell_print(sh, "    GPIO%d: IE=%d PUE=%d PDE=%d OD=%d",
                    pin,
                    (val >> 6) & 1,
                    (val >> 3) & 1,
                    (val >> 2) & 1,
                    (val >> 7) & 1);
    }

    /* Restore PIO control */
    pio_gpio_init(pio, PROBE_SWCLK_PIN);
    pio_gpio_init(pio, PROBE_SWDIO_PIN);
    gpio_pull_up(PROBE_SWDIO_PIN);  /* Idle state is HIGH */
    pio_sm_set_consecutive_pindirs(pio, sm, PROBE_PIN_OFFSET, 2, true);
    if (was_enabled) {
        pio_sm_set_enabled(pio, sm, true);
    }

    /* PIO-based read test */
    shell_print(sh, "\n  5. PIO Read Test:");
    uint32_t read1 = pio_swd_read_bits(8);
    shell_print(sh, "    Read 8 bits: 0x%02x (%s)",
                read1,
                read1 == 0xFF ? "all HIGH - target idle/pullup" :
                read1 == 0x00 ? "all LOW - check connection" : "mixed");

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pio,
    SHELL_CMD(all, NULL, "Show all PIO blocks overview", cmd_pio_all),
    SHELL_CMD(status, NULL, "Show PIO SWD state machine status", cmd_pio_status),
    SHELL_CMD(debug, NULL, "Show PIO SWD debug registers", cmd_pio_debug),
    SHELL_CMD(reset, NULL, "Clear PIO SWD debug flags", cmd_pio_reset),
    SHELL_CMD(test, NULL, "Test PIO state machine (write/read cycle)", cmd_pio_test),
    SHELL_CMD(loopback, NULL, "Test level shifter direction", cmd_pio_loopback),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pio, &sub_pio, "PIO diagnostics (RP2040)", NULL);

#endif /* CONFIG_SOC_RP2040 */
