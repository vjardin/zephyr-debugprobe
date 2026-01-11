/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - CMSIS-DAP Implementation
 *
 * This implements the CMSIS-DAP v2 protocol for debug access.
 * Reference: https://arm-software.github.io/CMSIS_5/DAP/html/index.html
 *
 * Port from FreeRTOS/pico-sdk to Zephyr RTOS
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <string.h>

#include "probe_config.h"
#include "DAP_config.h"
#include "probe.h"
#include "dap.h"
#include "swo.h"

#ifdef CONFIG_SOC_RP2040
#include <hardware/gpio.h>
#endif

LOG_MODULE_REGISTER(dap, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * DAP command/status definitions are provided by DAP_config.h
 * (included via probe.h). Local definitions only for items not
 * covered there.
 */

/* DAP Port Auto-detect (alias for disabled at connect time) */
#define DAP_PORT_AUTODETECT     0x00

/* DAP Host Status LED types */
#define DAP_STATUS_CONNECT      0
#define DAP_STATUS_RUNNING      1

/* Transfer configuration */
static uint8_t idle_cycles = 0;
static uint16_t retry_count = 100;
static uint16_t match_retry = 0;
static uint32_t match_mask = 0xFFFFFFFF;

/* SWD configuration */
static uint8_t swd_turnaround = 1;  /* Default: 1 cycle (0 = 1 cycle per spec) */
static uint8_t swd_data_phase = 0;  /* Default: no data phase on WAIT/FAULT */

/* Current port */
static uint8_t dap_port = DAP_PORT_DISABLED;

/* Transfer timestamp (captured when DAP_TRANSFER_TIMESTAMP is set) */
static uint32_t transfer_timestamp = 0;

/* LED state tracking */
static bool led_connect_state = false;
static bool led_running_state = false;

/* Statistics counters */
static volatile uint32_t stats_transfers = 0;
static volatile uint32_t stats_transfer_ok = 0;
static volatile uint32_t stats_transfer_wait = 0;
static volatile uint32_t stats_transfer_fault = 0;
static volatile uint32_t stats_transfer_error = 0;
static volatile uint32_t stats_retries = 0;

/* DAP command counters - helps identify what pyocd is sending */
static volatile uint32_t cmd_info = 0;
static volatile uint32_t cmd_connect = 0;
static volatile uint32_t cmd_disconnect = 0;
static volatile uint32_t cmd_transfer = 0;
static volatile uint32_t cmd_transfer_block = 0;
static volatile uint32_t cmd_swj_sequence = 0;
static volatile uint32_t cmd_swj_clock = 0;
static volatile uint32_t cmd_swj_pins = 0;
static volatile uint32_t cmd_swd_configure = 0;
static volatile uint32_t cmd_transfer_configure = 0;
static volatile uint32_t cmd_other = 0;
static volatile uint32_t cmd_total = 0;

/* Last command tracking for debugging */
static volatile uint8_t last_transfer_req_count = 0;  /* Transfer requested count */
static volatile uint8_t last_transfer_resp_count = 0; /* Transfer response count */
static volatile uint8_t last_transfer_ack = 0;
static volatile uint16_t last_block_req_count = 0;   /* TransferBlock requested count */
static volatile uint16_t last_block_resp_count = 0;  /* TransferBlock response count */
static volatile uint8_t last_block_ack = 0;

/* Protocol tracing */
static bool dap_trace_enabled = false;

/* Dormant-to-SWD wakeup sequence (for ADIv5.2+ targets like Cortex-M33) */
static const uint8_t dormant_wakeup_sequence[] = {
    0x00,  /* 8 SWCLK cycles with SWDIO LOW */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* 64 bits HIGH */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* 64 bits HIGH (128 total) */
    0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,  /* Selection Alert sequence */
    0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
    0xa0,  /* 4 bits LOW, 4 bits of activation code (0x0A) */
};

/* JTAG-to-SWD switch sequence (standard ARM sequence) */
static const uint8_t jtag_to_swd_sequence[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* 56 bits of 1s (line reset) */
    0x9e, 0xe7,  /* JTAG-to-SWD: 0111 1001 1110 0111 (16 bits, LSB first) */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* 56 bits of 1s (line reset) */
    0x00  /* 8 idle cycles */
};

/*
 * Get DAP Info String
 */
static uint32_t dap_info_string(uint8_t id, uint8_t *info)
{
    const char *str;
    uint32_t len;

    switch (id) {
    case DAP_ID_VENDOR:
        str = DAP_VENDOR;
        break;
    case DAP_ID_PRODUCT:
        str = DAP_PRODUCT;
        break;
    case DAP_ID_SER_NUM:
        /* Serial number is handled elsewhere */
        str = "0001";
        break;
    case DAP_ID_FW_VER:
        str = DAP_FW_VER;
        break;
    case DAP_ID_DEVICE_VENDOR:
        str = "";
        break;
    case DAP_ID_DEVICE_NAME:
        str = "";
        break;
    default:
        info[0] = 0;
        return 1;
    }

    len = strlen(str);
    if (len > 62) {
        len = 62;
    }
    info[0] = (uint8_t)(len + 1);
    memcpy(&info[1], str, len);
    info[len + 1] = '\0';
    return len + 2;
}

/*
 * DAP_Info command handler
 */
static uint32_t dap_cmd_info(const uint8_t *request, uint8_t *response)
{
    uint8_t id = request[1];

    response[0] = DAP_CMD_INFO;

    switch (id) {
    case DAP_ID_CAPABILITIES:
        /* Capabilities byte 0:
         * Bit 0: SWD supported
         * Bit 1: JTAG supported
         * Bit 2: SWO UART mode supported
         * Bit 3: SWO Manchester mode supported
         * Bit 4: Atomic commands supported
         * Bit 5: Timestamp clock supported
         * Bit 6: SWO Streaming Trace supported
         */
        response[1] = 1;  /* Length */
        response[2] = 0x17; /* SWD + JTAG + SWO_UART + Atomic */
        return 3;

    case DAP_ID_TIMESTAMP_CLOCK:
        /* Timestamp clock frequency */
        response[1] = 4;
        response[2] = (CONFIG_SYS_CLOCK_TICKS_PER_SEC >> 0) & 0xFF;
        response[3] = (CONFIG_SYS_CLOCK_TICKS_PER_SEC >> 8) & 0xFF;
        response[4] = (CONFIG_SYS_CLOCK_TICKS_PER_SEC >> 16) & 0xFF;
        response[5] = (CONFIG_SYS_CLOCK_TICKS_PER_SEC >> 24) & 0xFF;
        return 6;

    case DAP_ID_SWO_BUFFER_SIZE:
        /* SWO buffer size */
        {
            uint32_t buf_size = swo_get_buffer_size();
            response[1] = 4;
            response[2] = (uint8_t)(buf_size >> 0);
            response[3] = (uint8_t)(buf_size >> 8);
            response[4] = (uint8_t)(buf_size >> 16);
            response[5] = (uint8_t)(buf_size >> 24);
        }
        return 6;

    case DAP_ID_PACKET_COUNT:
        /* Maximum packet count */
        response[1] = 1;
        response[2] = DAP_PACKET_COUNT;
        return 3;

    case DAP_ID_PACKET_SIZE:
        /* Maximum packet size */
        response[1] = 2;
        response[2] = (DAP_PACKET_SIZE >> 0) & 0xFF;
        response[3] = (DAP_PACKET_SIZE >> 8) & 0xFF;
        return 4;

    default:
        /* String info */
        return 1 + dap_info_string(id, &response[1]);
    }
}

/*
 * DAP_HostStatus command handler
 */
static uint32_t dap_cmd_host_status(const uint8_t *request, uint8_t *response)
{
    uint8_t type = request[1];
    uint8_t status = request[2];

    response[0] = DAP_CMD_HOST_STATUS;
    response[1] = DAP_OK;

    switch (type) {
    case DAP_STATUS_CONNECT:
        led_connect_state = (status != 0);
        /* TODO: Control connected LED */
        break;
    case DAP_STATUS_RUNNING:
        led_running_state = (status != 0);
        /* TODO: Control running LED */
        break;
    default:
        break;
    }

    return 2;
}

/*
 * DAP_Connect command handler
 *
 * For ADIv5.2+ targets (like Cortex-M33 with APPROTECT), we send:
 * 1. Dormant wakeup sequence - wakes target from dormant state
 * 2. JTAG-to-SWD sequence - standard switch sequence
 * This enables pyocd and other tools to connect without special handling.
 */
static uint32_t dap_cmd_connect(const uint8_t *request, uint8_t *response)
{
    uint8_t port = request[1];

    response[0] = DAP_CMD_CONNECT;

    if (port == DAP_PORT_AUTODETECT) {
        port = DAP_PORT_SWD;  /* Default to SWD */
    }

    switch (port) {
    case DAP_PORT_SWD:
        /* Initialize SWD - match original debugprobe behavior exactly */
        probe_port_enable(true);
        probe_pins_active();
        dap_port = DAP_PORT_SWD;
        response[1] = DAP_PORT_SWD;
        LOG_INF("DAP connected (SWD)");
        break;
    case DAP_PORT_JTAG:
        /* Initialize JTAG */
        probe_jtag_init();
        probe_port_enable(true);
        dap_port = DAP_PORT_JTAG;
        response[1] = DAP_PORT_JTAG;
        LOG_INF("DAP connected (JTAG)");
        break;
    default:
        dap_port = DAP_PORT_DISABLED;
        response[1] = DAP_PORT_DISABLED;
        break;
    }

    return 2;
}

/*
 * DAP_Disconnect command handler
 */
static uint32_t dap_cmd_disconnect(const uint8_t *request, uint8_t *response)
{
    ARG_UNUSED(request);

    response[0] = DAP_CMD_DISCONNECT;
    response[1] = DAP_OK;

    probe_pins_hiz();
    probe_port_enable(false);
    dap_port = DAP_PORT_DISABLED;

    LOG_INF("DAP disconnected");
    return 2;
}

/*
 * DAP_TransferConfigure command handler
 */
static uint32_t dap_cmd_transfer_configure(const uint8_t *request, uint8_t *response)
{
    idle_cycles = request[1];
    retry_count = request[2] | ((uint16_t)request[3] << 8);
    match_retry = request[4] | ((uint16_t)request[5] << 8);

    response[0] = DAP_CMD_TRANSFER_CONFIGURE;
    response[1] = DAP_OK;

    return 2;
}

/*
 * DAP_Transfer command handler
 *
 * Supports match operations:
 * - DAP_TRANSFER_MATCH_MASK (0x20): Set the match mask for subsequent compares
 * - DAP_TRANSFER_MATCH_VALUE (0x10): Compare read data against match value
 */
static uint32_t dap_cmd_transfer(const uint8_t *request, uint8_t *response)
{
    uint32_t request_count;
    uint32_t request_value;
    uint32_t match_value = 0;
    uint32_t response_count = 0;
    const uint8_t *req_ptr;
    uint8_t *resp_ptr;
    uint8_t dap_index;
    uint8_t req_byte;
    uint8_t ack = DAP_TRANSFER_OK;
    uint32_t retry;
    uint32_t match_retry_count;
    uint32_t data;
    uint32_t post_read = 0;
    uint32_t check_write = 0;

    response[0] = DAP_CMD_TRANSFER;

    dap_index = request[1];
    request_count = request[2];
    req_ptr = &request[3];
    resp_ptr = &response[3];

    if (dap_port == DAP_PORT_DISABLED) {
        goto end;
    }

    while (request_count--) {
        req_byte = *req_ptr++;

        /* Check for match mask operation */
        if (req_byte & DAP_TRANSFER_MATCH_MASK) {
            /* Read and set the match mask */
            match_mask = (uint32_t)req_ptr[0] |
                        ((uint32_t)req_ptr[1] << 8) |
                        ((uint32_t)req_ptr[2] << 16) |
                        ((uint32_t)req_ptr[3] << 24);
            req_ptr += 4;
            response_count++;
            continue;
        }

        /* Get request value if write */
        if (!(req_byte & DAP_TRANSFER_RnW)) {
            request_value = (uint32_t)req_ptr[0] |
                           ((uint32_t)req_ptr[1] << 8) |
                           ((uint32_t)req_ptr[2] << 16) |
                           ((uint32_t)req_ptr[3] << 24);
            req_ptr += 4;
        }

        /* Get match value if match operation */
        if (req_byte & DAP_TRANSFER_MATCH_VALUE) {
            match_value = (uint32_t)req_ptr[0] |
                         ((uint32_t)req_ptr[1] << 8) |
                         ((uint32_t)req_ptr[2] << 16) |
                         ((uint32_t)req_ptr[3] << 24);
            req_ptr += 4;
        }

        /* Process posted read - get data from previous AP read */
        if (post_read) {
            /* Check if this is a consecutive AP read (without match value) */
            if ((req_byte & (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) == DAP_TRANSFER_APnDP) {
                /* Consecutive AP read: send next read AND get previous data */
                retry = retry_count;
                do {
                    ack = probe_swd_transfer(req_byte, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                if (ack != DAP_TRANSFER_OK) {
                    goto end;
                }

                /* Store previous AP read data */
                *resp_ptr++ = (uint8_t)data;
                *resp_ptr++ = (uint8_t)(data >> 8);
                *resp_ptr++ = (uint8_t)(data >> 16);
                *resp_ptr++ = (uint8_t)(data >> 24);

                /* post_read stays 1, and we skip the main transfer section */
                response_count++;
                continue;
            } else {
                /* Not an AP read - read DP RDBUFF to get last AP value */
                retry = retry_count;
                do {
                    ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_RDBUFF, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                if (ack != DAP_TRANSFER_OK) {
                    goto end;
                }

                *resp_ptr++ = (uint8_t)data;
                *resp_ptr++ = (uint8_t)(data >> 8);
                *resp_ptr++ = (uint8_t)(data >> 16);
                *resp_ptr++ = (uint8_t)(data >> 24);
                post_read = 0;
            }
        }

        /* Process write response */
        if (check_write) {
            /* Read DP CTRL/STAT for sticky bits */
            retry = retry_count;
            do {
                ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_CTRL_STAT, &data);
            } while ((ack == DAP_TRANSFER_WAIT) && retry--);

            if (ack != DAP_TRANSFER_OK) {
                goto end;
            }

            if (data & (DAP_SWD_STICKYERR | DAP_SWD_STICKYCMP | DAP_SWD_STICKYORUN)) {
                ack = DAP_TRANSFER_ERROR;
                goto end;
            }
            check_write = 0;
        }

        /* Execute transfer */
        if (req_byte & DAP_TRANSFER_RnW) {
            /* Read operation */
            if (req_byte & DAP_TRANSFER_MATCH_VALUE) {
                /* Match operation - retry until value matches or timeout */
                match_retry_count = match_retry;
                do {
                    /* Read the register */
                    retry = retry_count;
                    do {
                        ack = probe_swd_transfer(req_byte & 0x0F, &data);
                    } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                    if (ack != DAP_TRANSFER_OK) {
                        goto end;
                    }

                    /* For AP read, need to read RDBUFF */
                    if (req_byte & DAP_TRANSFER_APnDP) {
                        retry = retry_count;
                        do {
                            ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_RDBUFF, &data);
                        } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                        if (ack != DAP_TRANSFER_OK) {
                            goto end;
                        }
                    }

                    /* Check if value matches */
                    if ((data & match_mask) == (match_value & match_mask)) {
                        break;
                    }
                } while (match_retry_count--);

                /* Check if match succeeded */
                if ((data & match_mask) != (match_value & match_mask)) {
                    ack = DAP_TRANSFER_MISMATCH;
                    goto end;
                }
            } else if (req_byte & DAP_TRANSFER_APnDP) {
                /* AP read - post read */
                retry = retry_count;
                do {
                    ack = probe_swd_transfer(req_byte, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                if (ack != DAP_TRANSFER_OK) {
                    goto end;
                }
                post_read = 1;
            } else {
                /* DP read - immediate */
                retry = retry_count;
                do {
                    ack = probe_swd_transfer(req_byte, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                if (ack != DAP_TRANSFER_OK) {
                    goto end;
                }

                *resp_ptr++ = (uint8_t)data;
                *resp_ptr++ = (uint8_t)(data >> 8);
                *resp_ptr++ = (uint8_t)(data >> 16);
                *resp_ptr++ = (uint8_t)(data >> 24);
            }
        } else {
            /* Write */
            retry = retry_count;
            do {
                ack = probe_swd_transfer(req_byte, &request_value);
            } while ((ack == DAP_TRANSFER_WAIT) && retry--);

            if (ack != DAP_TRANSFER_OK) {
                goto end;
            }

            check_write = 1;
        }

        response_count++;

        /* Capture timestamp if requested */
        if (req_byte & DAP_TRANSFER_TIMESTAMP) {
            transfer_timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
        }
    }

    /* Final post read */
    if (post_read) {
        retry = retry_count;
        do {
            ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_RDBUFF, &data);
        } while ((ack == DAP_TRANSFER_WAIT) && retry--);

        if (ack != DAP_TRANSFER_OK) {
            goto end;
        }

        *resp_ptr++ = (uint8_t)data;
        *resp_ptr++ = (uint8_t)(data >> 8);
        *resp_ptr++ = (uint8_t)(data >> 16);
        *resp_ptr++ = (uint8_t)(data >> 24);
    }

end:
    /* Update statistics - ensure Total = OK + WAIT + FAULT + ERROR */
    if (ack == DAP_TRANSFER_OK) {
        stats_transfers += response_count;
        stats_transfer_ok += response_count;
    } else {
        /* response_count = successful transfers before failure
         * Add 1 for the failed transfer itself */
        stats_transfers += response_count + 1;
        stats_transfer_ok += response_count;
        if (ack == DAP_TRANSFER_WAIT) {
            stats_transfer_wait++;
        } else if (ack == DAP_TRANSFER_FAULT) {
            stats_transfer_fault++;
        } else {
            stats_transfer_error++;
        }
    }

    response[1] = (uint8_t)response_count;
    response[2] = ack;

    return (resp_ptr - response);
}

/*
 * DAP_SWJ_Pins command handler
 *
 * Pin mapping:
 * - Bit 0: SWCLK/TCK (directly controlled)
 * - Bit 1: SWDIO/TMS (directly controlled)
 * - Bit 2: TDI (output to target)
 * - Bit 3: TDO (input from target)
 * - Bit 5: nTRST (active low JTAG reset)
 * - Bit 7: nRESET (active low target reset)
 */
static uint32_t dap_cmd_swj_pins(const uint8_t *request, uint8_t *response)
{
    uint8_t value = request[1];
    uint8_t select = request[2];
    uint32_t wait_us = (uint32_t)request[3] |
                       ((uint32_t)request[4] << 8) |
                       ((uint32_t)request[5] << 16) |
                       ((uint32_t)request[6] << 24);
    uint8_t pin_state;

    response[0] = DAP_CMD_SWJ_PINS;

    /* Set pins according to select mask */
    if (select & (1 << 0)) {
        /* SWCLK/TCK */
        probe_write_bit((value >> 0) & 1);  /* Uses SWCLK internally */
    }
    if (select & (1 << 1)) {
        /* SWDIO/TMS */
        probe_write_bit((value >> 1) & 1);  /* Sets SWDIO */
    }
    if (select & (1 << 7)) {
        /* nRESET - active low */
        probe_set_reset((value & (1 << 7)) == 0);
    }

    /* Wait if requested (with timeout limit) */
    if (wait_us > 0) {
        if (wait_us > 3000000) {
            wait_us = 3000000;  /* Max 3 seconds */
        }
        k_busy_wait(wait_us);
    }

    /* Read current pin state */
    pin_state = 0;

    /* Bit 0: SWCLK/TCK - read back (typically output, reads last written value) */
    /* For GPIO bit-bang, we can read the output latch */

    /* Bit 1: SWDIO/TMS - can be input or output */
    if (probe_read_bit()) {
        pin_state |= (1 << 1);
    }

    /* Bit 3: TDO - input from target (only on legacy GPIO path) */
#if PROBE_JTAG_TDO_AVAILABLE
    if (gpio_dev != NULL) {
        if (gpio_pin_get(gpio_dev, PROBE_TDO_PIN)) {
            pin_state |= (1 << 3);
        }
    }
#endif

    /* Bit 7: nRESET - read back (open-drain, can be pulled low by target) */
#ifdef PROBE_RESET_PIN
    if (gpio_dev != NULL && PROBE_RESET_PIN >= 0) {
        if (gpio_pin_get(gpio_dev, PROBE_RESET_PIN)) {
            pin_state |= (1 << 7);
        }
    }
#endif

    response[1] = pin_state;
    return 2;
}

/*
 * DAP_SWJ_Clock command handler
 */
static uint32_t dap_cmd_swj_clock(const uint8_t *request, uint8_t *response)
{
    uint32_t clock = (uint32_t)request[1] |
                     ((uint32_t)request[2] << 8) |
                     ((uint32_t)request[3] << 16) |
                     ((uint32_t)request[4] << 24);

    response[0] = DAP_CMD_SWJ_CLOCK;

    if (clock == 0) {
        response[1] = DAP_ERROR;
    } else {
        probe_set_swj_clock(clock);
        response[1] = DAP_OK;
    }

    return 2;
}

/*
 * DAP_SWJ_Sequence command handler - match original debugprobe exactly
 */
static uint32_t dap_cmd_swj_sequence(const uint8_t *request, uint8_t *response)
{
    uint32_t count = request[1];

    response[0] = DAP_CMD_SWJ_SEQUENCE;

    if (count == 0) {
        count = 256;
    }

    probe_swj_sequence(count, &request[2]);
    response[1] = DAP_OK;

    return 2;
}

/*
 * DAP_SWD_Configure command handler
 *
 * Configuration byte format:
 *   Bits 0-1: Turnaround period (0=1 cycle, 1=2, 2=3, 3=4 cycles)
 *   Bit 2: DataPhase (0=no data phase on WAIT/FAULT, 1=always data phase)
 */
static uint32_t dap_cmd_swd_configure(const uint8_t *request, uint8_t *response)
{
    uint8_t cfg = request[1];

    response[0] = DAP_CMD_SWD_CONFIGURE;
    response[1] = DAP_OK;

    /* Extract turnaround period: 0-3 means 1-4 cycles */
    swd_turnaround = (cfg & 0x03) + 1;

    /* Extract data phase flag */
    swd_data_phase = (cfg >> 2) & 0x01;

    return 2;
}

/*
 * DAP_Delay command handler
 */
static uint32_t dap_cmd_delay(const uint8_t *request, uint8_t *response)
{
    uint16_t delay_us = (uint16_t)request[1] | ((uint16_t)request[2] << 8);

    response[0] = DAP_CMD_DELAY;
    response[1] = DAP_OK;

    if (delay_us > 0) {
        k_busy_wait(delay_us);
    }

    return 2;
}

/*
 * DAP_WriteAbort command handler
 */
static uint32_t dap_cmd_write_abort(const uint8_t *request, uint8_t *response)
{
    uint8_t dap_index = request[1];
    uint32_t abort_val = (uint32_t)request[2] |
                         ((uint32_t)request[3] << 8) |
                         ((uint32_t)request[4] << 16) |
                         ((uint32_t)request[5] << 24);

    ARG_UNUSED(dap_index);

    response[0] = DAP_CMD_WRITE_ABORT;

    if (dap_port == DAP_PORT_DISABLED) {
        response[1] = DAP_ERROR;
        return 2;
    }

    /* Write to DP ABORT register (address 0, write) */
    uint8_t ack = probe_swd_transfer(0x00, &abort_val);
    response[1] = (ack == DAP_TRANSFER_OK) ? DAP_OK : DAP_ERROR;

    return 2;
}

/*
 * DAP_TransferBlock command handler
 * Performs multiple read/write transfers to a single register
 */
static uint32_t dap_cmd_transfer_block(const uint8_t *request, uint8_t *response)
{
    uint8_t dap_index;
    uint16_t transfer_count;
    uint8_t req_byte;
    const uint8_t *req_ptr;
    uint8_t *resp_ptr;
    uint16_t response_count = 0;
    uint8_t ack = DAP_TRANSFER_OK;
    uint32_t retry;
    uint32_t data;

    response[0] = DAP_CMD_TRANSFER_BLOCK;

    dap_index = request[1];
    transfer_count = (uint16_t)request[2] | ((uint16_t)request[3] << 8);
    req_byte = request[4];
    req_ptr = &request[5];
    resp_ptr = &response[4];

    ARG_UNUSED(dap_index);

    if (dap_port == DAP_PORT_DISABLED) {
        goto end;
    }

    if (req_byte & DAP_TRANSFER_RnW) {
        /* Read block */
        while (transfer_count--) {
            /* First read is dummy for AP reads */
            if (response_count == 0 && (req_byte & DAP_TRANSFER_APnDP)) {
                retry = retry_count;
                do {
                    ack = probe_swd_transfer(req_byte, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);

                if (ack != DAP_TRANSFER_OK) {
                    goto end;
                }
            }

            /* Read data */
            retry = retry_count;
            if (transfer_count == 0 && (req_byte & DAP_TRANSFER_APnDP)) {
                /* Last AP read - read RDBUFF */
                do {
                    ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_RDBUFF, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);
            } else {
                do {
                    ack = probe_swd_transfer(req_byte, &data);
                } while ((ack == DAP_TRANSFER_WAIT) && retry--);
            }

            if (ack != DAP_TRANSFER_OK) {
                goto end;
            }

            *resp_ptr++ = (uint8_t)data;
            *resp_ptr++ = (uint8_t)(data >> 8);
            *resp_ptr++ = (uint8_t)(data >> 16);
            *resp_ptr++ = (uint8_t)(data >> 24);
            response_count++;
        }
    } else {
        /* Write block */
        while (transfer_count--) {
            data = (uint32_t)req_ptr[0] |
                   ((uint32_t)req_ptr[1] << 8) |
                   ((uint32_t)req_ptr[2] << 16) |
                   ((uint32_t)req_ptr[3] << 24);
            req_ptr += 4;

            retry = retry_count;
            do {
                ack = probe_swd_transfer(req_byte, &data);
            } while ((ack == DAP_TRANSFER_WAIT) && retry--);

            if (ack != DAP_TRANSFER_OK) {
                goto end;
            }
            response_count++;
        }

        /* Check for sticky errors after write block */
        retry = retry_count;
        do {
            ack = probe_swd_transfer(DAP_TRANSFER_RnW | DAP_DP_CTRL_STAT, &data);
        } while ((ack == DAP_TRANSFER_WAIT) && retry--);

        if (ack == DAP_TRANSFER_OK) {
            if (data & (DAP_SWD_STICKYERR | DAP_SWD_STICKYCMP | DAP_SWD_STICKYORUN)) {
                ack = DAP_TRANSFER_ERROR;
            }
        }
    }

end:
    /* Update statistics - ensure Total = OK + WAIT + FAULT + ERROR */
    if (ack == DAP_TRANSFER_OK) {
        stats_transfers += response_count;
        stats_transfer_ok += response_count;
    } else {
        /* response_count = successful transfers before failure
         * Add 1 for the failed transfer itself */
        stats_transfers += response_count + 1;
        stats_transfer_ok += response_count;
        if (ack == DAP_TRANSFER_WAIT) {
            stats_transfer_wait++;
        } else if (ack == DAP_TRANSFER_FAULT) {
            stats_transfer_fault++;
        } else {
            stats_transfer_error++;
        }
    }

    response[1] = (uint8_t)response_count;
    response[2] = (uint8_t)(response_count >> 8);
    response[3] = ack;

    return (resp_ptr - response);
}

/*
 * DAP_TransferAbort command handler
 * Aborts current transfer (sets flag checked by Transfer/TransferBlock)
 */
static volatile bool transfer_abort = false;

static uint32_t dap_cmd_transfer_abort(const uint8_t *request, uint8_t *response)
{
    ARG_UNUSED(request);

    transfer_abort = true;

    response[0] = DAP_CMD_TRANSFER_ABORT;
    response[1] = DAP_OK;

    return 2;
}

/*
 * DAP_JTAG_Sequence command handler
 * Execute JTAG sequences
 */
static uint32_t dap_cmd_jtag_sequence(const uint8_t *request, uint8_t *response)
{
    uint8_t sequence_count;
    const uint8_t *req_ptr;
    uint8_t *resp_ptr;

    response[0] = DAP_CMD_JTAG_SEQUENCE;

    if (dap_port != DAP_PORT_JTAG) {
        response[1] = DAP_ERROR;
        return 2;
    }

    sequence_count = request[1];
    req_ptr = &request[2];
    resp_ptr = &response[2];

    while (sequence_count--) {
        uint8_t seq_info = *req_ptr++;
        uint32_t seq_count = seq_info & 0x3F;
        uint32_t bytes;

        if (seq_count == 0) {
            seq_count = 64;
        }
        bytes = (seq_count + 7) / 8;

        /* Execute sequence */
        probe_jtag_sequence(seq_info, req_ptr, resp_ptr);

        /* Move pointers */
        req_ptr += bytes;  /* TDI data consumed */
        if (seq_info & 0x80) {
            resp_ptr += bytes;  /* TDO data captured */
        }
    }

    response[1] = DAP_OK;
    return (resp_ptr - response);
}

/*
 * DAP_JTAG_Configure command handler
 * Configure JTAG chain
 */
static uint32_t dap_cmd_jtag_configure(const uint8_t *request, uint8_t *response)
{
    uint8_t count = request[1];
    const uint8_t *ir_length = &request[2];

    response[0] = DAP_CMD_JTAG_CONFIGURE;

    probe_jtag_configure(count, ir_length);
    response[1] = DAP_OK;

    return 2;
}

/*
 * DAP_JTAG_IDCODE command handler
 * Read JTAG IDCODE from device
 */
static uint32_t dap_cmd_jtag_idcode(const uint8_t *request, uint8_t *response)
{
    uint8_t index = request[1];
    uint32_t idcode;
    int ret;

    response[0] = DAP_CMD_JTAG_IDCODE;

    if (dap_port != DAP_PORT_JTAG) {
        response[1] = DAP_ERROR;
        return 2;
    }

    ret = probe_jtag_read_idcode(index, &idcode);
    if (ret < 0) {
        response[1] = DAP_ERROR;
        return 2;
    }

    response[1] = DAP_OK;
    response[2] = (uint8_t)(idcode >> 0);
    response[3] = (uint8_t)(idcode >> 8);
    response[4] = (uint8_t)(idcode >> 16);
    response[5] = (uint8_t)(idcode >> 24);

    return 6;
}

/*
 * DAP_SWD_Sequence command handler
 * Execute SWD sequences with precise control
 */
static uint32_t dap_cmd_swd_sequence(const uint8_t *request, uint8_t *response)
{
    uint8_t sequence_count;
    const uint8_t *req_ptr;
    uint8_t *resp_ptr;
    uint32_t seq_info;
    uint32_t seq_count;
    uint32_t n;

    response[0] = DAP_CMD_SWD_SEQUENCE;

    if (dap_port != DAP_PORT_SWD) {
        response[1] = DAP_ERROR;
        return 2;
    }

    sequence_count = request[1];
    req_ptr = &request[2];
    resp_ptr = &response[2];

    while (sequence_count--) {
        seq_info = *req_ptr++;
        seq_count = seq_info & 0x3F;  /* Bits 0-5: count (0 = 64) */
        if (seq_count == 0) {
            seq_count = 64;
        }

        if (seq_info & 0x80) {
            /* Input sequence (read) */
            uint32_t bit_idx;
            uint8_t byte_val = 0;

            /* Read bits from SWDIO */
            for (n = 0; n < seq_count; n++) {
                bit_idx = n % 8;
                if (bit_idx == 0 && n > 0) {
                    /* Store completed byte */
                    *resp_ptr++ = byte_val;
                    byte_val = 0;
                }
                /* Read one bit with clock cycle */
                if (probe_read_bit()) {
                    byte_val |= (1U << bit_idx);
                }
            }
            /* Store final byte */
            *resp_ptr++ = byte_val;
        } else {
            /* Output sequence (write) */
            uint32_t bytes = (seq_count + 7) / 8;
            probe_swj_sequence(seq_count, req_ptr);
            req_ptr += bytes;
        }
    }

    response[1] = DAP_OK;
    return (resp_ptr - response);
}

/*
 * DAP_ResetTarget command handler
 */
static uint32_t dap_cmd_reset_target(const uint8_t *request, uint8_t *response)
{
    ARG_UNUSED(request);

    response[0] = DAP_CMD_RESET_TARGET;
    response[1] = DAP_OK;
    response[2] = 0;  /* Execute not implemented */

    /* Pulse reset */
    probe_set_reset(true);
    k_busy_wait(1000);  /* 1ms reset pulse */
    probe_set_reset(false);

    return 3;
}

/*
 * SWO Commands
 */

/*
 * DAP_SWO_Transport command handler
 */
static uint32_t dap_cmd_swo_transport(const uint8_t *request, uint8_t *response)
{
    uint8_t transport = request[1];

    response[0] = DAP_CMD_SWO_TRANSPORT;

    if (swo_set_transport(transport) == 0) {
        response[1] = DAP_OK;
    } else {
        response[1] = DAP_ERROR;
    }

    return 2;
}

/*
 * DAP_SWO_Mode command handler
 */
static uint32_t dap_cmd_swo_mode(const uint8_t *request, uint8_t *response)
{
    uint8_t mode = request[1];

    response[0] = DAP_CMD_SWO_MODE;

    if (swo_set_mode(mode) == 0) {
        response[1] = DAP_OK;
    } else {
        response[1] = DAP_ERROR;
    }

    return 2;
}

/*
 * DAP_SWO_Baudrate command handler
 */
static uint32_t dap_cmd_swo_baudrate(const uint8_t *request, uint8_t *response)
{
    uint32_t baudrate = (uint32_t)request[1] |
                        ((uint32_t)request[2] << 8) |
                        ((uint32_t)request[3] << 16) |
                        ((uint32_t)request[4] << 24);
    uint32_t actual;

    response[0] = DAP_CMD_SWO_BAUDRATE;

    actual = swo_set_baudrate(baudrate);

    response[1] = (uint8_t)(actual >> 0);
    response[2] = (uint8_t)(actual >> 8);
    response[3] = (uint8_t)(actual >> 16);
    response[4] = (uint8_t)(actual >> 24);

    return 5;
}

/*
 * DAP_SWO_Control command handler
 */
static uint32_t dap_cmd_swo_control(const uint8_t *request, uint8_t *response)
{
    uint8_t control = request[1];

    response[0] = DAP_CMD_SWO_CONTROL;

    if (control & 1) {
        /* Start capture */
        if (swo_start() == 0) {
            response[1] = DAP_OK;
        } else {
            response[1] = DAP_ERROR;
        }
    } else {
        /* Stop capture */
        swo_stop();
        response[1] = DAP_OK;
    }

    return 2;
}

/*
 * DAP_SWO_Status command handler
 */
static uint32_t dap_cmd_swo_status(const uint8_t *request, uint8_t *response)
{
    uint32_t count;

    ARG_UNUSED(request);

    response[0] = DAP_CMD_SWO_STATUS;
    response[1] = swo_get_status();

    count = swo_get_count();
    response[2] = (uint8_t)(count >> 0);
    response[3] = (uint8_t)(count >> 8);
    response[4] = (uint8_t)(count >> 16);
    response[5] = (uint8_t)(count >> 24);

    return 6;
}

/*
 * DAP_SWO_ExtendedStatus command handler
 */
static uint32_t dap_cmd_swo_ext_status(const uint8_t *request, uint8_t *response)
{
    uint8_t control = request[1];
    uint32_t count;
    uint8_t *resp_ptr = &response[1];

    response[0] = DAP_CMD_SWO_EXT_STATUS;

    if (control & 0x01) {
        /* Return trace status */
        *resp_ptr++ = swo_get_status();
    }

    if (control & 0x02) {
        /* Return trace count */
        count = swo_get_count();
        *resp_ptr++ = (uint8_t)(count >> 0);
        *resp_ptr++ = (uint8_t)(count >> 8);
        *resp_ptr++ = (uint8_t)(count >> 16);
        *resp_ptr++ = (uint8_t)(count >> 24);
    }

    if (control & 0x04) {
        /* Return timestamp (microseconds) */
        *resp_ptr++ = (uint8_t)(transfer_timestamp >> 0);
        *resp_ptr++ = (uint8_t)(transfer_timestamp >> 8);
        *resp_ptr++ = (uint8_t)(transfer_timestamp >> 16);
        *resp_ptr++ = (uint8_t)(transfer_timestamp >> 24);
    }

    return (resp_ptr - response);
}

/*
 * DAP_SWO_Data command handler
 */
static uint32_t dap_cmd_swo_data(const uint8_t *request, uint8_t *response)
{
    uint16_t max_count = (uint16_t)request[1] | ((uint16_t)request[2] << 8);
    uint32_t count;

    response[0] = DAP_CMD_SWO_DATA;
    response[1] = swo_get_status();

    /* Limit to packet size */
    if (max_count > DAP_PACKET_SIZE - 4) {
        max_count = DAP_PACKET_SIZE - 4;
    }

    count = swo_read_data(&response[4], max_count);

    response[2] = (uint8_t)(count >> 0);
    response[3] = (uint8_t)(count >> 8);

    return 4 + count;
}

/*
 * Command queue for DAP_QueueCommands/DAP_ExecuteCommands
 */
#define DAP_QUEUE_SIZE 1024
static uint8_t cmd_queue[DAP_QUEUE_SIZE];
static uint32_t cmd_queue_len = 0;

/*
 * Calculate command length based on command type and parameters
 * Returns the full length of the command including all parameters
 */
static uint32_t dap_get_command_length(const uint8_t *cmd_ptr, uint32_t max_len)
{
    if (max_len < 1) {
        return 0;
    }

    uint8_t cmd = cmd_ptr[0];
    uint32_t len = 1;  /* Command byte */

    switch (cmd) {
    case DAP_CMD_INFO:
        len = 2;  /* cmd + id */
        break;

    case DAP_CMD_HOST_STATUS:
        len = 3;  /* cmd + type + status */
        break;

    case DAP_CMD_CONNECT:
        len = 2;  /* cmd + port */
        break;

    case DAP_CMD_DISCONNECT:
        len = 1;  /* cmd only */
        break;

    case DAP_CMD_TRANSFER_CONFIGURE:
        len = 6;  /* cmd + idle + retry(2) + match_retry(2) */
        break;

    case DAP_CMD_TRANSFER:
        if (max_len >= 3) {
            /* cmd + index + count + variable data */
            uint8_t count = cmd_ptr[2];
            len = 3;  /* Base length */
            /* Each transfer request is at least 1 byte, plus 4 for write data */
            for (uint8_t i = 0; i < count && len < max_len; i++) {
                uint8_t req = cmd_ptr[len];
                len++;  /* Request byte */
                if (req & DAP_TRANSFER_MATCH_MASK) {
                    len += 4;  /* Match mask value */
                } else if (!(req & DAP_TRANSFER_RnW)) {
                    len += 4;  /* Write data */
                }
                if (req & DAP_TRANSFER_MATCH_VALUE) {
                    len += 4;  /* Match value */
                }
            }
        }
        break;

    case DAP_CMD_TRANSFER_BLOCK:
        if (max_len >= 5) {
            /* cmd + index + count(2) + request + data */
            uint16_t count = cmd_ptr[2] | ((uint16_t)cmd_ptr[3] << 8);
            uint8_t req = cmd_ptr[4];
            len = 5;  /* cmd + index + count(2) + request */
            if (!(req & DAP_TRANSFER_RnW)) {
                len += count * 4;  /* Write data */
            }
        }
        break;

    case DAP_CMD_TRANSFER_ABORT:
        len = 1;  /* cmd only */
        break;

    case DAP_CMD_WRITE_ABORT:
        len = 6;  /* cmd + index + abort_value(4) */
        break;

    case DAP_CMD_DELAY:
        len = 3;  /* cmd + delay(2) */
        break;

    case DAP_CMD_RESET_TARGET:
        len = 1;  /* cmd only */
        break;

    case DAP_CMD_SWJ_PINS:
        len = 7;  /* cmd + value + select + wait(4) */
        break;

    case DAP_CMD_SWJ_CLOCK:
        len = 5;  /* cmd + clock(4) */
        break;

    case DAP_CMD_SWJ_SEQUENCE:
        if (max_len >= 2) {
            /* cmd + count + data */
            uint32_t bits = cmd_ptr[1];
            if (bits == 0) bits = 256;
            len = 2 + ((bits + 7) / 8);
        }
        break;

    case DAP_CMD_SWD_CONFIGURE:
        len = 2;  /* cmd + config */
        break;

    case DAP_CMD_SWD_SEQUENCE:
        if (max_len >= 2) {
            /* cmd + count + sequences */
            uint8_t seq_count = cmd_ptr[1];
            len = 2;
            for (uint8_t i = 0; i < seq_count && len < max_len; i++) {
                uint8_t seq_info = cmd_ptr[len];
                uint32_t bits = seq_info & 0x3F;
                if (bits == 0) bits = 64;
                len++;  /* seq_info */
                if (!(seq_info & 0x80)) {
                    len += (bits + 7) / 8;  /* Output data */
                }
            }
        }
        break;

    case DAP_CMD_JTAG_SEQUENCE:
        if (max_len >= 2) {
            /* cmd + count + sequences */
            uint8_t seq_count = cmd_ptr[1];
            len = 2;
            for (uint8_t i = 0; i < seq_count && len < max_len; i++) {
                uint8_t seq_info = cmd_ptr[len];
                uint32_t bits = seq_info & 0x3F;
                if (bits == 0) bits = 64;
                len++;  /* seq_info */
                len += (bits + 7) / 8;  /* TDI data */
            }
        }
        break;

    case DAP_CMD_JTAG_CONFIGURE:
        if (max_len >= 2) {
            len = 2 + cmd_ptr[1];  /* cmd + count + ir_lengths */
        }
        break;

    case DAP_CMD_JTAG_IDCODE:
        len = 2;  /* cmd + index */
        break;

    case DAP_CMD_SWO_TRANSPORT:
    case DAP_CMD_SWO_MODE:
        len = 2;  /* cmd + value */
        break;

    case DAP_CMD_SWO_BAUDRATE:
        len = 5;  /* cmd + baudrate(4) */
        break;

    case DAP_CMD_SWO_CONTROL:
        len = 2;  /* cmd + control */
        break;

    case DAP_CMD_SWO_STATUS:
        len = 1;  /* cmd only */
        break;

    case DAP_CMD_SWO_EXT_STATUS:
        len = 2;  /* cmd + control */
        break;

    case DAP_CMD_SWO_DATA:
        len = 3;  /* cmd + count(2) */
        break;

    case DAP_CMD_QUEUE_COMMANDS:
    case DAP_CMD_EXECUTE_COMMANDS:
        len = 2;  /* cmd + count (execute has no params) */
        break;

    default:
        /* Vendor commands or unknown - assume minimal */
        len = 2;
        break;
    }

    return (len <= max_len) ? len : max_len;
}

/*
 * DAP_QueueCommands command handler
 * Queues multiple commands for later execution
 */
static uint32_t dap_cmd_queue_commands(const uint8_t *request, uint8_t *response)
{
    uint8_t cmd_count = request[1];
    const uint8_t *cmd_ptr = &request[2];
    uint32_t remaining = DAP_PACKET_SIZE - 2;  /* Estimate of remaining data */

    response[0] = DAP_CMD_QUEUE_COMMANDS;

    while (cmd_count > 0 && remaining > 0) {
        uint32_t cmd_len = dap_get_command_length(cmd_ptr, remaining);

        if (cmd_len == 0) {
            response[1] = DAP_ERROR;
            return 2;
        }

        if (cmd_queue_len + cmd_len > DAP_QUEUE_SIZE) {
            response[1] = DAP_ERROR;
            return 2;
        }

        memcpy(&cmd_queue[cmd_queue_len], cmd_ptr, cmd_len);
        cmd_queue_len += cmd_len;
        cmd_ptr += cmd_len;
        remaining -= cmd_len;
        cmd_count--;
    }

    response[1] = DAP_OK;
    return 2;
}

/*
 * DAP_ExecuteCommands command handler
 * Executes all queued commands
 */
static uint32_t dap_cmd_execute_commands(const uint8_t *request, uint8_t *response)
{
    ARG_UNUSED(request);

    uint8_t *resp_ptr = &response[1];
    uint32_t queue_pos = 0;
    uint8_t temp_response[DAP_PACKET_SIZE];

    response[0] = DAP_CMD_EXECUTE_COMMANDS;

    while (queue_pos < cmd_queue_len) {
        uint32_t cmd_response_len;
        uint32_t cmd_len;

        /* Get the length of current command */
        cmd_len = dap_get_command_length(&cmd_queue[queue_pos],
                                         cmd_queue_len - queue_pos);
        if (cmd_len == 0) {
            break;  /* Invalid command, stop processing */
        }

        /* Process each queued command */
        cmd_response_len = dap_process_request(&cmd_queue[queue_pos],
                                               cmd_len,
                                               temp_response,
                                               sizeof(temp_response));

        /* Copy response (skip command byte as we're concatenating) */
        if ((resp_ptr - response) + cmd_response_len < DAP_PACKET_SIZE) {
            memcpy(resp_ptr, temp_response, cmd_response_len);
            resp_ptr += cmd_response_len;
        }

        /* Move to next command using proper length */
        queue_pos += cmd_len;
    }

    /* Clear queue */
    cmd_queue_len = 0;

    return (resp_ptr - response);
}

/*
 * Initialize DAP
 */
void dap_init(void)
{
    dap_port = DAP_PORT_DISABLED;
    idle_cycles = 0;
    retry_count = 100;
    match_retry = 0;
    match_mask = 0xFFFFFFFF;

    LOG_INF("DAP initialized");
}

/*
 * Process DAP request
 */
uint32_t dap_process_request(const uint8_t *request, uint32_t request_len,
                             uint8_t *response, uint32_t response_max)
{
    uint32_t response_len = 0;
    uint8_t cmd = request[0];

    ARG_UNUSED(request_len);
    ARG_UNUSED(response_max);

    cmd_total++;

    /* Trace every command if enabled */
    if (dap_trace_enabled) {
        if (cmd == DAP_CMD_TRANSFER) {
            LOG_INF("DAP cmd=0x%02x (Transfer) count=%u", cmd, request[2]);
        } else if (cmd == DAP_CMD_TRANSFER_BLOCK) {
            uint16_t count = (uint16_t)request[2] | ((uint16_t)request[3] << 8);
            LOG_INF("DAP cmd=0x%02x (TransferBlock) count=%u req=0x%02x",
                    cmd, count, request[4]);
        } else {
            LOG_INF("DAP cmd=0x%02x", cmd);
        }
    }

    switch (cmd) {
    case DAP_CMD_INFO:
        cmd_info++;
        response_len = dap_cmd_info(request, response);
        break;
    case DAP_CMD_HOST_STATUS:
        cmd_other++;
        response_len = dap_cmd_host_status(request, response);
        break;
    case DAP_CMD_CONNECT:
        cmd_connect++;
        response_len = dap_cmd_connect(request, response);
        break;
    case DAP_CMD_DISCONNECT:
        cmd_disconnect++;
        response_len = dap_cmd_disconnect(request, response);
        break;
    case DAP_CMD_TRANSFER_CONFIGURE:
        cmd_transfer_configure++;
        response_len = dap_cmd_transfer_configure(request, response);
        break;
    case DAP_CMD_TRANSFER:
        cmd_transfer++;
        /* Track request count before processing */
        last_transfer_req_count = request[2];
        response_len = dap_cmd_transfer(request, response);
        /* Track response count after processing */
        last_transfer_resp_count = response[1];
        last_transfer_ack = response[2];
        /* Record for USB packet trace */
        usb_dap_trace_transfer(last_transfer_req_count, last_transfer_resp_count,
                               last_transfer_ack);
        break;
    case DAP_CMD_TRANSFER_BLOCK:
        cmd_transfer_block++;
        /* Track request count before processing */
        last_block_req_count = (uint16_t)request[2] | ((uint16_t)request[3] << 8);
        response_len = dap_cmd_transfer_block(request, response);
        /* Track response count after processing */
        last_block_resp_count = (uint16_t)response[1] | ((uint16_t)response[2] << 8);
        last_block_ack = response[3];
        /* Debug: log if mismatch */
        if (last_block_req_count != last_block_resp_count && dap_trace_enabled) {
            LOG_WRN("TransferBlock: req=%u resp=%u ack=%u resp_len=%u",
                    last_block_req_count, last_block_resp_count,
                    last_block_ack, response_len);
        }
        break;
    case DAP_CMD_TRANSFER_ABORT:
        cmd_other++;
        response_len = dap_cmd_transfer_abort(request, response);
        break;
    case DAP_CMD_WRITE_ABORT:
        cmd_other++;
        response_len = dap_cmd_write_abort(request, response);
        break;
    case DAP_CMD_DELAY:
        cmd_other++;
        response_len = dap_cmd_delay(request, response);
        break;
    case DAP_CMD_SWJ_PINS:
        cmd_swj_pins++;
        response_len = dap_cmd_swj_pins(request, response);
        break;
    case DAP_CMD_SWJ_CLOCK:
        cmd_swj_clock++;
        response_len = dap_cmd_swj_clock(request, response);
        break;
    case DAP_CMD_SWJ_SEQUENCE:
        cmd_swj_sequence++;
        response_len = dap_cmd_swj_sequence(request, response);
        break;
    case DAP_CMD_SWD_CONFIGURE:
        cmd_swd_configure++;
        response_len = dap_cmd_swd_configure(request, response);
        break;
    case DAP_CMD_JTAG_SEQUENCE:
        cmd_other++;
        response_len = dap_cmd_jtag_sequence(request, response);
        break;
    case DAP_CMD_JTAG_CONFIGURE:
        cmd_other++;
        response_len = dap_cmd_jtag_configure(request, response);
        break;
    case DAP_CMD_JTAG_IDCODE:
        cmd_other++;
        response_len = dap_cmd_jtag_idcode(request, response);
        break;
    case DAP_CMD_SWD_SEQUENCE:
        cmd_other++;
        response_len = dap_cmd_swd_sequence(request, response);
        break;
    case DAP_CMD_RESET_TARGET:
        cmd_other++;
        response_len = dap_cmd_reset_target(request, response);
        break;
    case DAP_CMD_SWO_TRANSPORT:
        cmd_other++;
        response_len = dap_cmd_swo_transport(request, response);
        break;
    case DAP_CMD_SWO_MODE:
        cmd_other++;
        response_len = dap_cmd_swo_mode(request, response);
        break;
    case DAP_CMD_SWO_BAUDRATE:
        cmd_other++;
        response_len = dap_cmd_swo_baudrate(request, response);
        break;
    case DAP_CMD_SWO_CONTROL:
        cmd_other++;
        response_len = dap_cmd_swo_control(request, response);
        break;
    case DAP_CMD_SWO_STATUS:
        cmd_other++;
        response_len = dap_cmd_swo_status(request, response);
        break;
    case DAP_CMD_SWO_EXT_STATUS:
        cmd_other++;
        response_len = dap_cmd_swo_ext_status(request, response);
        break;
    case DAP_CMD_SWO_DATA:
        cmd_other++;
        response_len = dap_cmd_swo_data(request, response);
        break;
    case DAP_CMD_QUEUE_COMMANDS:
        cmd_other++;
        response_len = dap_cmd_queue_commands(request, response);
        break;
    case DAP_CMD_EXECUTE_COMMANDS:
        cmd_other++;
        response_len = dap_cmd_execute_commands(request, response);
        break;
    default:
        cmd_other++;
        /* Unknown command */
        if (cmd >= DAP_CMD_VENDOR_BASE) {
            /* Vendor command - pass to vendor handler */
            response_len = dap_vendor_command(request, response);
        } else {
            response[0] = cmd;
            response[1] = DAP_ERROR;
            response_len = 2;
        }
        break;
    }

    /* Trace response if enabled */
    if (dap_trace_enabled && response_len > 0) {
        if (cmd == DAP_CMD_TRANSFER) {
            LOG_INF("  -> resp_len=%u count=%u ack=%u",
                    response_len, response[1], response[2]);
        } else if (cmd == DAP_CMD_TRANSFER_BLOCK) {
            uint16_t count = (uint16_t)response[1] | ((uint16_t)response[2] << 8);
            LOG_INF("  -> resp_len=%u count=%u ack=%u",
                    response_len, count, response[3]);
        }
    }

    return response_len;
}

/*
 * Get configured idle cycles count
 */
uint8_t dap_get_idle_cycles(void)
{
    return idle_cycles;
}

/*
 * Get configured SWD turnaround period (1-4 cycles)
 */
uint8_t dap_get_swd_turnaround(void)
{
    return swd_turnaround;
}

/*
 * Get configured SWD data phase flag
 */
uint8_t dap_get_swd_data_phase(void)
{
    return swd_data_phase;
}

/*
 * Shell Commands for DAP diagnostics
 */

static const char *dap_port_str(uint8_t port)
{
    switch (port) {
    case DAP_PORT_SWD:
        return "SWD";
    case DAP_PORT_JTAG:
        return "JTAG";
    default:
        return "Disabled";
    }
}

/* External USB stats function */
extern void usb_dap_get_stats(uint32_t *out_pkts, uint32_t *out_zlp, uint32_t *in_pkts,
                              uint32_t *in_bytes, uint32_t *errors, uint32_t *disabled_drops,
                              uint8_t *last_cmd, uint8_t *last_resp,
                              uint32_t *last_req_len, uint32_t *last_resp_len);
extern void usb_dap_get_ringbuf_stats(uint32_t *req_wptr, uint32_t *req_rptr,
                                      uint32_t *resp_wptr, uint32_t *resp_rptr,
                                      uint32_t *req_full, uint32_t *resp_empty,
                                      uint32_t *max_pending);
extern void usb_dap_reset_stats(void);
extern void usb_dap_trace_enable(bool enable);
extern void usb_dap_dump_trace(void);
extern void usb_dap_trace_transfer(uint8_t req_count, uint8_t resp_count, uint8_t ack);

/* dap stats - show DAP status and transfer statistics */
static int cmd_dap_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint32_t usb_out, usb_zlp, usb_in, usb_bytes, usb_err, usb_drops;
    uint8_t last_cmd_usb, last_resp_usb;
    uint32_t last_req_len, last_resp_len;

    usb_dap_get_stats(&usb_out, &usb_zlp, &usb_in, &usb_bytes, &usb_err, &usb_drops,
                      &last_cmd_usb, &last_resp_usb, &last_req_len, &last_resp_len);

    shell_print(sh, "DAP Status:");
    shell_print(sh, "  Port: %s", dap_port_str(dap_port));
    shell_print(sh, "  Clock: %u Hz", probe_get_swj_clock());
    shell_print(sh, "  SWD: turnaround=%u cycles, data_phase=%s",
                swd_turnaround, swd_data_phase ? "on" : "off");
    shell_print(sh, "  Transfer: idle=%u cycles, retry=%u, match_retry=%u",
                idle_cycles, retry_count, match_retry);
    shell_print(sh, "  Connected: %s", led_connect_state ? "yes" : "no");
    shell_print(sh, "  Running: %s", led_running_state ? "yes" : "no");
    shell_print(sh, "  Trace: %s", dap_trace_enabled ? "on" : "off");

    shell_print(sh, "USB Layer:");
    shell_print(sh, "  OUT packets: %u (ZLP: %u)", usb_out, usb_zlp);
    shell_print(sh, "  IN packets: %u (%u bytes)", usb_in, usb_bytes);
    shell_print(sh, "  Errors: %u, Disabled drops: %u", usb_err, usb_drops);
    shell_print(sh, "  Last: cmd=0x%02x req_len=%u, resp=0x%02x resp_len=%u",
                last_cmd_usb, last_req_len, last_resp_usb, last_resp_len);
    shell_print(sh, "  OUT-IN balance: %d", (int)(usb_out - usb_zlp - usb_in));

    /* Ring buffer stats */
    uint32_t req_wptr, req_rptr, resp_wptr, resp_rptr;
    uint32_t req_full, resp_empty, max_pending;
    usb_dap_get_ringbuf_stats(&req_wptr, &req_rptr, &resp_wptr, &resp_rptr,
                              &req_full, &resp_empty, &max_pending);
    shell_print(sh, "Ring Buffer:");
    shell_print(sh, "  Request: w=%u r=%u pending=%u (was_full=%u)",
                req_wptr, req_rptr, req_wptr - req_rptr, req_full);
    shell_print(sh, "  Response: w=%u r=%u pending=%u (was_empty=%u)",
                resp_wptr, resp_rptr, resp_wptr - resp_rptr, resp_empty);
    shell_print(sh, "  Max pending: %u", max_pending);

    shell_print(sh, "DAP Commands:");
    shell_print(sh, "  Total: %u", cmd_total);
    shell_print(sh, "  Info: %u, Connect: %u, Disconnect: %u",
                cmd_info, cmd_connect, cmd_disconnect);
    shell_print(sh, "  Transfer: %u, TransferBlock: %u",
                cmd_transfer, cmd_transfer_block);
    shell_print(sh, "  SWJ_Seq: %u, SWJ_Clock: %u, SWJ_Pins: %u",
                cmd_swj_sequence, cmd_swj_clock, cmd_swj_pins);
    shell_print(sh, "  SWD_Cfg: %u, Transfer_Cfg: %u, Other: %u",
                cmd_swd_configure, cmd_transfer_configure, cmd_other);

    shell_print(sh, "Last Transfer:");
    shell_print(sh, "  DAP_Transfer: req=%u resp=%u ack=%u%s",
                last_transfer_req_count, last_transfer_resp_count, last_transfer_ack,
                (last_transfer_req_count != last_transfer_resp_count) ? " MISMATCH!" : "");
    shell_print(sh, "  DAP_TransferBlock: req=%u resp=%u ack=%u%s",
                last_block_req_count, last_block_resp_count, last_block_ack,
                (last_block_req_count != last_block_resp_count) ? " MISMATCH!" : "");

    shell_print(sh, "SWD Transfers:");
    shell_print(sh, "  Total: %u", stats_transfers);
    shell_print(sh, "  OK: %u", stats_transfer_ok);
    shell_print(sh, "  WAIT: %u (exhausted retries)", stats_transfer_wait);
    shell_print(sh, "  FAULT: %u", stats_transfer_fault);
    shell_print(sh, "  ERROR: %u", stats_transfer_error);
    shell_print(sh, "  Retries: %u", stats_retries);

    return 0;
}

/* dap reset - reset statistics counters */
static int cmd_dap_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* Reset SWD transfer stats */
    stats_transfers = 0;
    stats_transfer_ok = 0;
    stats_transfer_wait = 0;
    stats_transfer_fault = 0;
    stats_transfer_error = 0;
    stats_retries = 0;

    /* Reset DAP command counters */
    cmd_info = 0;
    cmd_connect = 0;
    cmd_disconnect = 0;
    cmd_transfer = 0;
    cmd_transfer_block = 0;
    cmd_swj_sequence = 0;
    cmd_swj_clock = 0;
    cmd_swj_pins = 0;
    cmd_swd_configure = 0;
    cmd_transfer_configure = 0;
    cmd_other = 0;
    cmd_total = 0;

    /* Reset last transfer tracking */
    last_transfer_req_count = 0;
    last_transfer_resp_count = 0;
    last_transfer_ack = 0;
    last_block_req_count = 0;
    last_block_resp_count = 0;
    last_block_ack = 0;

    /* Reset USB stats */
    usb_dap_reset_stats();

    shell_print(sh, "All statistics reset");
    return 0;
}

/* dap clock - show clock configuration */
static int cmd_dap_clock(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Clock Configuration:");
    shell_print(sh, "  System: %u Hz", probe_get_sys_clock());
    shell_print(sh, "  SWCLK: %u Hz", probe_get_swj_clock());
    shell_print(sh, "  PIO: %s", probe_is_pio_enabled() ? "enabled" : "disabled (GPIO bit-bang)");

    return 0;
}

/* dap pins - show SWD pin states */
static int cmd_dap_pins(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int swclk, swdio, reset;
    int swclk_pin, swdio_pin, reset_pin;

    probe_get_pin_state(&swclk, &swdio, &reset);
    probe_get_pin_info(&swclk_pin, &swdio_pin, &reset_pin);

    shell_print(sh, "SWD Pin States:");
    shell_print(sh, "  SWCLK (GPIO%d): %s", swclk_pin,
                swclk < 0 ? "N/A" : (swclk ? "high" : "low"));
    shell_print(sh, "  SWDIO (GPIO%d): %s", swdio_pin,
                swdio < 0 ? "N/A" : (swdio ? "high" : "low"));
    if (reset_pin >= 0) {
        shell_print(sh, "  nRESET (GPIO%d): %s", reset_pin,
                    reset < 0 ? "N/A" : (reset ? "released" : "asserted"));
    } else {
        shell_print(sh, "  nRESET: not available");
    }

    return 0;
}

/* dap trace - enable/disable protocol tracing */
static int cmd_dap_trace(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Trace: %s", dap_trace_enabled ? "on" : "off");
        shell_print(sh, "Usage: dap trace <on|off>");
        return 0;
    }

    if (strcmp(argv[1], "on") == 0) {
        dap_trace_enabled = true;
        usb_dap_trace_enable(true);  /* Also enable USB packet trace */
        shell_print(sh, "DAP trace enabled (including USB packet trace)");
    } else if (strcmp(argv[1], "off") == 0) {
        dap_trace_enabled = false;
        usb_dap_trace_enable(false);
        shell_print(sh, "DAP trace disabled");
    } else {
        shell_print(sh, "Usage: dap trace <on|off>");
        return -EINVAL;
    }

    return 0;
}

/* dap dump - dump USB packet trace */
static int cmd_dap_dump(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Dumping USB packet trace to log...");
    usb_dap_dump_trace();
    return 0;
}

/* dap hwreset - toggle hardware reset pin */
static int cmd_dap_hwreset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Hardware Reset:");
    shell_print(sh, "  Asserting reset...");
    probe_set_reset(true);
    k_msleep(100);
    shell_print(sh, "  Releasing reset...");
    probe_set_reset(false);
    k_msleep(100);
    shell_print(sh, "  Done");
    return 0;
}

/* dap pio - toggle PIO/GPIO mode */
static int cmd_dap_pio(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "PIO mode: %s", probe_is_pio_enabled() ? "enabled" : "disabled (GPIO bit-bang)");
        shell_print(sh, "Usage: dap pio <on|off>");
        return 0;
    }

    if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0) {
        probe_set_pio_enabled(true);
        shell_print(sh, "PIO mode: %s", probe_is_pio_enabled() ? "enabled" : "failed to enable");
    } else if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0) {
        probe_set_pio_enabled(false);
        shell_print(sh, "PIO mode: disabled (GPIO bit-bang)");
    } else {
        shell_print(sh, "Usage: dap pio <on|off>");
    }

    return 0;
}

/* dap rawtest - detailed SWD debug with bit-by-bit trace */
static int cmd_dap_rawtest(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "SWD Raw Transaction Debug:");

    /* Disable PIO, force GPIO bit-bang for debugging */
    bool was_pio = probe_is_pio_enabled();
    probe_set_pio_enabled(false);
    shell_print(sh, "  Using GPIO bit-bang mode");

    /* Enable port */
    probe_port_enable(true);

    /* Set very slow clock for debugging */
    probe_set_swj_clock(10000);  /* 10kHz */
    shell_print(sh, "  Clock: 10 kHz (very slow for debug)");

    /* First check pin states in current mode */
    int swclk, swdio, reset;
    probe_get_pin_state(&swclk, &swdio, &reset);
    shell_print(sh, "  Initial pins (output mode): SWCLK=%d SWDIO=%d nRST=%d", swclk, swdio, reset);

    /* Check what's actually on the bus by switching to input */
    shell_print(sh, "  Reading bus (input mode)...");
    probe_read_bit();  /* Force switch to input */
    probe_get_pin_state(&swclk, &swdio, &reset);
    shell_print(sh, "  Bus state (before any SWD): SWDIO=%d", swdio);

#ifdef CONFIG_SOC_RP2040
    /* Debug Probe: read from SWDI (GPIO13) which is separate input path */
    int swdi_val = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "  SWDI (input pin): %d", swdi_val);
    if (swdio == 0 || swdi_val == 0) {
        shell_print(sh, "  WARNING: Line is LOW before we start!");
        shell_print(sh, "  (Target may be driving or not connected)");
    }
#else
    if (swdio == 0) {
        shell_print(sh, "  WARNING: SWDIO LOW before we start!");
    }
#endif

    /* Send line reset sequence manually - 56 clocks with SWDIO high */
    shell_print(sh, "  Sending line reset (56 clocks with SWDIO HIGH)...");
    uint8_t reset_seq[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    probe_swj_sequence(56, reset_seq);

    /* Check SWDIO state after reset */
    probe_get_pin_state(&swclk, &swdio, &reset);
#ifdef CONFIG_SOC_RP2040
    int sdk_swdio = gpio_get(PROBE_SWDIO_PIN);
    int sdk_swdi = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "  After line reset: SWDIO(out)=%d, SWDI(in)=%d",
                sdk_swdio, sdk_swdi);

    /* Check GPIO function and try to fix */
    uint32_t gpio_ctrl = *(volatile uint32_t *)(0x40014000 + PROBE_SWDIO_PIN * 8 + 4);
    shell_print(sh, "  GPIO%d function: %d (5=SIO, 6=PIO0, 7=PIO1)",
                PROBE_SWDIO_PIN, gpio_ctrl & 0x1f);

    /* Manually drive HIGH and check */
    gpio_put(PROBE_SWDIO_PIN, 1);
    k_busy_wait(10);
    shell_print(sh, "  After manual drive HIGH: SDK=%d", gpio_get(PROBE_SWDIO_PIN));

    /* Try forcing back to SIO function */
    gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PROBE_SWDIO_PIN, GPIO_OUT);
    gpio_put(PROBE_SWDIO_PIN, 1);
    k_busy_wait(10);
    shell_print(sh, "  After force SIO + drive HIGH: SDK=%d", gpio_get(PROBE_SWDIO_PIN));
#else
    shell_print(sh, "  After line reset (output mode): SWDIO=%d", swdio);
#endif

    /* Now switch to INPUT mode and see what's actually on the bus */
    shell_print(sh, "  Switching to INPUT mode to read bus...");
    probe_read_bit();  /* This switches to input mode */
    probe_get_pin_state(&swclk, &swdio, &reset);
    shell_print(sh, "  Bus state (input mode): SWDIO=%d", swdio);

    /* Send JTAG-to-SWD switch */
    shell_print(sh, "  Sending JTAG-to-SWD (0xE79E)...");
    uint8_t switch_seq[] = {0x9e, 0xe7};
    probe_swj_sequence(16, switch_seq);

    /* Another line reset */
    probe_swj_sequence(56, reset_seq);

    /* Idle clocks */
    uint8_t idle[] = {0x00};
    probe_swj_sequence(8, idle);

    /* Check SWDIO again */
    probe_get_pin_state(&swclk, &swdio, &reset);
    shell_print(sh, "  After switch+reset: SWDIO=%d", swdio);

    /* Now try to read DPIDR with verbose output */
    shell_print(sh, "  Building DPIDR read request...");

    /* Request: Start=1, APnDP=0, RnW=1, A[2:3]=00, Parity=1, Stop=0, Park=1
     * Bit pattern: 1 0 1 00 1 0 1 = 0b10100101 = 0xA5
     */
    uint8_t req = 0xA5;
    shell_print(sh, "  Request byte: 0x%02X", req);
    shell_print(sh, "    Start=1, APnDP=0, RnW=1, A=00, Par=1, Stop=0, Park=1");

    /* Do the transfer and capture ACK */
    uint32_t dpidr = 0;
    uint32_t request = DAP_TRANSFER_RnW;  /* DP read, address 0 */
    uint8_t ack = probe_swd_transfer(request, &dpidr);

    shell_print(sh, "  ACK bits: %d%d%d (value=%u)",
                (ack >> 0) & 1, (ack >> 1) & 1, (ack >> 2) & 1, ack);

#ifdef CONFIG_SOC_RP2040
    /* Extra debug: check SWDI state right now */
    int swdi_after = gpio_get(PROBE_SWDI_PIN);
    shell_print(sh, "  SWDI after transfer: %d", swdi_after);
#endif

    if (ack == DAP_TRANSFER_OK) {
        shell_print(sh, "  DPIDR: 0x%08X - SUCCESS!", dpidr);
    } else if (ack == 0) {
        shell_print(sh, "  ACK=0: Target holding SWDIO LOW");
        shell_print(sh, "");
        shell_print(sh, "  Trying connect-under-reset...");

        /* Assert reset and hold it */
        probe_set_reset(true);
        k_msleep(10);

        /* Do full SWD init while in reset */
        probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);

        /* Try DPIDR while still in reset */
        ack = probe_swd_transfer(request, &dpidr);
        shell_print(sh, "  While in reset - ACK: %u", ack);

        if (ack == DAP_TRANSFER_OK) {
            shell_print(sh, "  DPIDR: 0x%08X - SUCCESS with connect-under-reset!", dpidr);
            /* Release reset */
            probe_set_reset(false);
        } else {
            /* Release reset and try dormant wakeup */
            probe_set_reset(false);
            k_msleep(10);

            shell_print(sh, "  Trying dormant wakeup sequence...");

            /* Hardware reset first */
            probe_set_reset(true);
            k_msleep(50);
            probe_set_reset(false);
            k_msleep(10);

            /* Send dormant wakeup */
            probe_swj_sequence(sizeof(dormant_wakeup_sequence) * 8, dormant_wakeup_sequence);

            /* Then JTAG-to-SWD */
            probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);

            /* Check bus state */
            probe_read_bit();
            probe_get_pin_state(&swclk, &swdio, &reset);
            shell_print(sh, "  After dormant wakeup: SWDIO=%d", swdio);

            /* Try DPIDR again */
            ack = probe_swd_transfer(request, &dpidr);
            shell_print(sh, "  Third attempt ACK: %u", ack);
            if (ack == DAP_TRANSFER_OK) {
                shell_print(sh, "  DPIDR: 0x%08X - SUCCESS with dormant wakeup!", dpidr);
            } else {
                shell_print(sh, "  Still failing. Target may have APPROTECT enabled.");
            }
        }
    } else if (ack == 7) {
        shell_print(sh, "  ACK=7: No response (SWDIO floating HIGH)");
        shell_print(sh, "  Target not responding - check connections");
    } else {
        shell_print(sh, "  Unexpected ACK=%u", ack);
    }

#ifdef CONFIG_SOC_RP2040
    /* Extra detailed debug: manual bit-bang with GPIO state at each step */
    shell_print(sh, "\n  Manual bit-bang debug:");

    /* Re-do JTAG-to-SWD sequence */
    probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);

    /* Set very slow clock (1 kHz) for manual debugging */
    probe_set_swj_clock(1000);

    /* Manual request: 0xA5 LSB first */
    gpio_set_function(PROBE_SWCLK_PIN, GPIO_FUNC_SIO);
    gpio_set_function(PROBE_SWDIO_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PROBE_SWCLK_PIN, GPIO_OUT);
    gpio_set_dir(PROBE_SWDIO_PIN, GPIO_OUT);
    gpio_put(PROBE_SWCLK_PIN, 0);

    shell_print(sh, "    Before request - SWDI=%d", gpio_get(PROBE_SWDI_PIN));

    /* Write 8 request bits manually */
    uint8_t req_bits = 0xA5;
    for (int i = 0; i < 8; i++) {
        gpio_put(PROBE_SWDIO_PIN, (req_bits >> i) & 1);
        k_busy_wait(100);
        gpio_put(PROBE_SWCLK_PIN, 1);
        k_busy_wait(100);
        gpio_put(PROBE_SWCLK_PIN, 0);
    }

    shell_print(sh, "    After request - SWDI=%d", gpio_get(PROBE_SWDI_PIN));

    /* Turnaround - tristate SWDIO */
    gpio_set_dir(PROBE_SWDIO_PIN, GPIO_IN);
    shell_print(sh, "    After tristate - SWDI=%d", gpio_get(PROBE_SWDI_PIN));

    k_busy_wait(100);
    gpio_put(PROBE_SWCLK_PIN, 1);  /* Turnaround rising */
    shell_print(sh, "    Turnaround HIGH - SWDI=%d", gpio_get(PROBE_SWDI_PIN));
    k_busy_wait(100);
    gpio_put(PROBE_SWCLK_PIN, 0);  /* Turnaround falling */
    shell_print(sh, "    Turnaround LOW - SWDI=%d", gpio_get(PROBE_SWDI_PIN));

    /* Read 3 ACK bits */
    uint8_t manual_ack = 0;
    for (int i = 0; i < 3; i++) {
        k_busy_wait(100);
        gpio_put(PROBE_SWCLK_PIN, 1);
        int bit = gpio_get(PROBE_SWDI_PIN);
        shell_print(sh, "    ACK[%d] on HIGH - SWDI=%d", i, bit);
        manual_ack |= (bit << i);
        k_busy_wait(100);
        gpio_put(PROBE_SWCLK_PIN, 0);
    }

    shell_print(sh, "    Manual ACK result: %u (0x%x)", manual_ack, manual_ack);
#endif

    /* Restore PIO mode if it was enabled */
    if (was_pio) {
        probe_set_pio_enabled(true);
    }

    return 0;
}

/* dap scan - try to detect target on SWD bus */
static int cmd_dap_scan(const struct shell *sh, size_t argc, char **argv)
{
    bool use_dormant = false;
    bool use_hwreset = false;

    /* Parse options */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--dormant") == 0) {
            use_dormant = true;
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            use_hwreset = true;
        }
    }

    uint32_t dpidr = 0;
    uint8_t ack;

    shell_print(sh, "SWD Target Scan:");

    /* Enable SWD port */
    probe_port_enable(true);
    shell_print(sh, "  Port enabled");

    /* Hardware reset if requested */
    if (use_hwreset) {
        shell_print(sh, "  Hardware reset...");
        probe_set_reset(true);
        k_msleep(50);
        probe_set_reset(false);
        k_msleep(50);
    }

    /* Set a slow clock for initial detection (100kHz) */
    probe_set_swj_clock(100000);
    shell_print(sh, "  Clock: 100 kHz (slow for detection)");

    /* Send dormant wakeup if requested */
    if (use_dormant) {
        shell_print(sh, "  Sending dormant wakeup sequence...");
        probe_swj_sequence(sizeof(dormant_wakeup_sequence) * 8, dormant_wakeup_sequence);
    }

    /* Send JTAG-to-SWD switch sequence */
    shell_print(sh, "  Sending JTAG-to-SWD sequence...");
    probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);

    /* Try to read DPIDR (Debug Port ID Register) */
    /* SWD read: APnDP=0 (DP), RnW=1 (Read), A[2:3]=0 (DPIDR at addr 0) */
    /* Request byte: Start=1, APnDP=0, RnW=1, A=00, Parity, Stop=0, Park=1 */
    shell_print(sh, "  Reading DPIDR...");

    /* Use the probe transfer function */
    uint32_t request = DAP_TRANSFER_RnW;  /* DP read, address 0 */
    ack = probe_swd_transfer(request, &dpidr);

    shell_print(sh, "  ACK: %u (%s)", ack,
                ack == 1 ? "OK" :
                ack == 2 ? "WAIT" :
                ack == 4 ? "FAULT" : "NO RESPONSE");

    if (ack == DAP_TRANSFER_OK) {
        shell_print(sh, "  DPIDR: 0x%08x", dpidr);
        shell_print(sh, "    Designer: 0x%03x", (dpidr >> 1) & 0x7FF);
        shell_print(sh, "    Version: %u", (dpidr >> 12) & 0xF);
        shell_print(sh, "    Revision: %u", (dpidr >> 28) & 0xF);
        shell_print(sh, "  Result: TARGET FOUND!");
        return 0;
    }

    /* Normal connection failed - try multiple recovery approaches */
    shell_print(sh, "  Normal connection failed (ACK=%u), trying recovery...", ack);

#ifdef CONFIG_SOC_RP2040
    /* Use GPIO bit-bang for recovery - more reliable for tricky targets */
    bool was_pio = probe_is_pio_enabled();
    if (was_pio) {
        probe_set_pio_enabled(false);
    }

    /* Try 1: Fresh JTAG-to-SWD sequence */
    shell_print(sh, "  Attempt 1: Fresh init sequence...");
    probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);
    ack = probe_swd_transfer(request, &dpidr);

    if (ack != DAP_TRANSFER_OK) {
        /* Try 2: Dormant wakeup + init */
        shell_print(sh, "  Attempt 2: Dormant wakeup...");
        probe_swj_sequence(sizeof(dormant_wakeup_sequence) * 8, dormant_wakeup_sequence);
        probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);
        ack = probe_swd_transfer(request, &dpidr);
    }

    if (ack != DAP_TRANSFER_OK) {
        /* Try 3: Multiple init sequences (like manual bit-bang) */
        shell_print(sh, "  Attempt 3: Multiple init sequences...");
        for (int i = 0; i < 3 && ack != DAP_TRANSFER_OK; i++) {
            probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);
            ack = probe_swd_transfer(request, &dpidr);
        }
    }

    if (ack == DAP_TRANSFER_OK) {
        shell_print(sh, "  Recovery SUCCESS!");
        shell_print(sh, "  DPIDR: 0x%08x", dpidr);
        shell_print(sh, "    Designer: 0x%03x", (dpidr >> 1) & 0x7FF);
        shell_print(sh, "    Version: %u", (dpidr >> 12) & 0xF);
        shell_print(sh, "    Revision: %u", (dpidr >> 28) & 0xF);

        /* Restore PIO mode */
        if (was_pio) {
            probe_set_pio_enabled(true);
        }
        return 0;
    }

    /* Restore PIO mode */
    if (was_pio) {
        probe_set_pio_enabled(true);
    }
#endif

    shell_print(sh, "  Result: No target detected (ACK=%u)", ack);
    shell_print(sh, "  Possible causes:");
    shell_print(sh, "    - Target not powered");
    shell_print(sh, "    - SWD lines not connected");
    shell_print(sh, "    - Wrong pins (check SWCLK/SWDIO)");
    shell_print(sh, "    - Target has debug protection enabled");

    return 0;
}

/* dap piotest - Compare PIO vs GPIO transfer modes */
static int cmd_dap_piotest(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

#ifndef CONFIG_SOC_RP2040
    shell_print(sh, "PIO test only available on RP2040");
    return 0;
#else
    uint32_t dpidr;
    uint8_t ack;
    uint32_t request = DAP_TRANSFER_RnW;  /* DP read DPIDR */

    shell_print(sh, "PIO vs GPIO Mode Comparison Test:");
    shell_print(sh, "==================================");

    /* First wake up the target using GPIO mode (known to work) */
    shell_print(sh, "\n1. Waking target with GPIO bit-bang...");
    probe_set_pio_enabled(false);
    probe_port_enable(true);
    probe_set_swj_clock(100000);

    /* Send multiple init sequences to wake up */
    probe_swj_sequence(sizeof(dormant_wakeup_sequence) * 8, dormant_wakeup_sequence);
    for (int i = 0; i < 3; i++) {
        probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);
    }
    ack = probe_swd_transfer(request, &dpidr);
    shell_print(sh, "   GPIO mode ACK: %u, DPIDR: 0x%08x", ack, dpidr);

    if (ack != DAP_TRANSFER_OK) {
        shell_print(sh, "   FAILED to wake target - cannot continue test");
        return -1;
    }
    shell_print(sh, "   Target awake!");

    /* Now test: Can we read with PIO immediately after GPIO woke it up? */
    shell_print(sh, "\n2. Switching to PIO mode (no re-init)...");
    probe_set_pio_enabled(true);

    /* Try reading without any new sequences - target should still be awake */
    ack = probe_swd_transfer(request, &dpidr);
    shell_print(sh, "   PIO read (no init): ACK=%u, DPIDR=0x%08x", ack, dpidr);

    if (ack == DAP_TRANSFER_OK) {
        shell_print(sh, "   SUCCESS! PIO works after GPIO wakeup");
    } else {
        shell_print(sh, "   FAILED! PIO cannot read even after GPIO wakeup");
        shell_print(sh, "   This suggests PIO has a fundamental issue");

        /* Try sending sequences via PIO */
        shell_print(sh, "\n3. Sending init via PIO...");
        probe_swj_sequence(sizeof(jtag_to_swd_sequence) * 8, jtag_to_swd_sequence);
        ack = probe_swd_transfer(request, &dpidr);
        shell_print(sh, "   PIO after PIO init: ACK=%u, DPIDR=0x%08x", ack, dpidr);
    }

    /* Test: Switch back to GPIO and verify it still works */
    shell_print(sh, "\n4. Back to GPIO mode...");
    probe_set_pio_enabled(false);
    ack = probe_swd_transfer(request, &dpidr);
    shell_print(sh, "   GPIO read: ACK=%u, DPIDR=0x%08x", ack, dpidr);

    /* Summary */
    shell_print(sh, "\n5. Diagnosis:");
    probe_set_pio_enabled(true);
    ack = probe_swd_transfer(request, &dpidr);
    uint8_t pio_ack = ack;

    probe_set_pio_enabled(false);
    ack = probe_swd_transfer(request, &dpidr);
    uint8_t gpio_ack = ack;

    shell_print(sh, "   PIO mode:  ACK=%u", pio_ack);
    shell_print(sh, "   GPIO mode: ACK=%u", gpio_ack);

    if (gpio_ack == DAP_TRANSFER_OK && pio_ack != DAP_TRANSFER_OK) {
        shell_print(sh, "\n   CONCLUSION: PIO transfer implementation has a bug!");
        shell_print(sh, "   GPIO works, PIO doesn't - issue is in pio_swd_transfer()");
    } else if (gpio_ack != DAP_TRANSFER_OK && pio_ack != DAP_TRANSFER_OK) {
        shell_print(sh, "\n   CONCLUSION: Target went back to sleep");
    } else {
        shell_print(sh, "\n   CONCLUSION: Both modes working!");
    }

    return 0;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dap,
    SHELL_CMD(stats, NULL, "Show DAP status and statistics", cmd_dap_stats),
    SHELL_CMD(reset, NULL, "Reset statistics counters", cmd_dap_reset),
    SHELL_CMD(hwreset, NULL, "Toggle hardware reset pin", cmd_dap_hwreset),
    SHELL_CMD(pio, NULL, "Enable/disable PIO mode (on|off)", cmd_dap_pio),
    SHELL_CMD(piotest, NULL, "Compare PIO vs GPIO transfer modes", cmd_dap_piotest),
    SHELL_CMD(clock, NULL, "Show clock configuration", cmd_dap_clock),
    SHELL_CMD(pins, NULL, "Show SWD pin states", cmd_dap_pins),
    SHELL_CMD(trace, NULL, "Enable/disable protocol tracing", cmd_dap_trace),
    SHELL_CMD(dump, NULL, "Dump USB packet trace to log", cmd_dap_dump),
    SHELL_CMD(scan, NULL, "Scan for SWD target [-r reset] [-d dormant]", cmd_dap_scan),
    SHELL_CMD(rawtest, NULL, "Detailed SWD debug with bit trace", cmd_dap_rawtest),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dap, &sub_dap, "DAP debug commands", NULL);
