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
        /* Initialize SWD */
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

        /* Process posted read */
        if (post_read) {
            if ((req_byte & DAP_TRANSFER_APnDP) && (req_byte & DAP_TRANSFER_RnW)) {
                /* Read AP register - continue post read */
            } else {
                /* Read DP RDBUFF */
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
    /* Update statistics */
    stats_transfers += response_count;
    if (ack == DAP_TRANSFER_OK) {
        stats_transfer_ok += response_count;
    } else if (ack == DAP_TRANSFER_WAIT) {
        stats_transfer_wait++;
    } else if (ack == DAP_TRANSFER_FAULT) {
        stats_transfer_fault++;
    } else {
        stats_transfer_error++;
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
 * DAP_SWJ_Sequence command handler
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
 */
static uint32_t dap_cmd_swd_configure(const uint8_t *request, uint8_t *response)
{
    uint8_t cfg = request[1];

    response[0] = DAP_CMD_SWD_CONFIGURE;
    response[1] = DAP_OK;

    /* Turnaround period (bits 0-1) and DataPhase (bit 2) */
    /* These are handled in the SWD implementation */
    ARG_UNUSED(cfg);

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
    /* Update statistics */
    stats_transfers += response_count;
    if (ack == DAP_TRANSFER_OK) {
        stats_transfer_ok += response_count;
    } else if (ack == DAP_TRANSFER_WAIT) {
        stats_transfer_wait++;
    } else if (ack == DAP_TRANSFER_FAULT) {
        stats_transfer_fault++;
    } else {
        stats_transfer_error++;
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

    switch (cmd) {
    case DAP_CMD_INFO:
        response_len = dap_cmd_info(request, response);
        break;
    case DAP_CMD_HOST_STATUS:
        response_len = dap_cmd_host_status(request, response);
        break;
    case DAP_CMD_CONNECT:
        response_len = dap_cmd_connect(request, response);
        break;
    case DAP_CMD_DISCONNECT:
        response_len = dap_cmd_disconnect(request, response);
        break;
    case DAP_CMD_TRANSFER_CONFIGURE:
        response_len = dap_cmd_transfer_configure(request, response);
        break;
    case DAP_CMD_TRANSFER:
        response_len = dap_cmd_transfer(request, response);
        break;
    case DAP_CMD_TRANSFER_BLOCK:
        response_len = dap_cmd_transfer_block(request, response);
        break;
    case DAP_CMD_TRANSFER_ABORT:
        response_len = dap_cmd_transfer_abort(request, response);
        break;
    case DAP_CMD_WRITE_ABORT:
        response_len = dap_cmd_write_abort(request, response);
        break;
    case DAP_CMD_DELAY:
        response_len = dap_cmd_delay(request, response);
        break;
    case DAP_CMD_SWJ_PINS:
        response_len = dap_cmd_swj_pins(request, response);
        break;
    case DAP_CMD_SWJ_CLOCK:
        response_len = dap_cmd_swj_clock(request, response);
        break;
    case DAP_CMD_SWJ_SEQUENCE:
        response_len = dap_cmd_swj_sequence(request, response);
        break;
    case DAP_CMD_SWD_CONFIGURE:
        response_len = dap_cmd_swd_configure(request, response);
        break;
    case DAP_CMD_JTAG_SEQUENCE:
        response_len = dap_cmd_jtag_sequence(request, response);
        break;
    case DAP_CMD_JTAG_CONFIGURE:
        response_len = dap_cmd_jtag_configure(request, response);
        break;
    case DAP_CMD_JTAG_IDCODE:
        response_len = dap_cmd_jtag_idcode(request, response);
        break;
    case DAP_CMD_SWD_SEQUENCE:
        response_len = dap_cmd_swd_sequence(request, response);
        break;
    case DAP_CMD_RESET_TARGET:
        response_len = dap_cmd_reset_target(request, response);
        break;
    case DAP_CMD_SWO_TRANSPORT:
        response_len = dap_cmd_swo_transport(request, response);
        break;
    case DAP_CMD_SWO_MODE:
        response_len = dap_cmd_swo_mode(request, response);
        break;
    case DAP_CMD_SWO_BAUDRATE:
        response_len = dap_cmd_swo_baudrate(request, response);
        break;
    case DAP_CMD_SWO_CONTROL:
        response_len = dap_cmd_swo_control(request, response);
        break;
    case DAP_CMD_SWO_STATUS:
        response_len = dap_cmd_swo_status(request, response);
        break;
    case DAP_CMD_SWO_EXT_STATUS:
        response_len = dap_cmd_swo_ext_status(request, response);
        break;
    case DAP_CMD_SWO_DATA:
        response_len = dap_cmd_swo_data(request, response);
        break;
    case DAP_CMD_QUEUE_COMMANDS:
        response_len = dap_cmd_queue_commands(request, response);
        break;
    case DAP_CMD_EXECUTE_COMMANDS:
        response_len = dap_cmd_execute_commands(request, response);
        break;
    default:
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

/* dap stats - show DAP status and transfer statistics */
static int cmd_dap_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "DAP Status:");
    shell_print(sh, "  Port: %s", dap_port_str(dap_port));
    shell_print(sh, "  Clock: %u Hz", probe_get_swj_clock());
    shell_print(sh, "  Connected: %s", led_connect_state ? "yes" : "no");
    shell_print(sh, "  Running: %s", led_running_state ? "yes" : "no");
    shell_print(sh, "Transfer Statistics:");
    shell_print(sh, "  Total: %u", stats_transfers);
    shell_print(sh, "  OK: %u", stats_transfer_ok);
    shell_print(sh, "  WAIT: %u", stats_transfer_wait);
    shell_print(sh, "  FAULT: %u", stats_transfer_fault);
    shell_print(sh, "  ERROR: %u", stats_transfer_error);

    return 0;
}

/* dap reset - reset statistics counters */
static int cmd_dap_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    stats_transfers = 0;
    stats_transfer_ok = 0;
    stats_transfer_wait = 0;
    stats_transfer_fault = 0;
    stats_transfer_error = 0;

    shell_print(sh, "Statistics reset");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dap,
    SHELL_CMD(stats, NULL, "Show DAP status and statistics", cmd_dap_stats),
    SHELL_CMD(reset, NULL, "Reset statistics counters", cmd_dap_reset),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dap, &sub_dap, "DAP debug commands", NULL);
