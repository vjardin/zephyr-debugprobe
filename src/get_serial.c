/* SPDX-License-Identifier: MIT
 * Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
 *
 * Debugprobe Zephyr Port - Serial Number Generation
 *
 * Generates a unique serial number from the device's unique ID.
 * Port from pico-sdk to Zephyr.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "get_serial.h"

LOG_MODULE_REGISTER(serial, CONFIG_LOG_DEFAULT_LEVEL);

/* For RP2040/RP2350, we can read the unique ID from flash */
#if defined(CONFIG_SOC_SERIES_RP2XXX) || defined(CONFIG_SOC_FAMILY_RPI_PICO)

#include <hardware/flash.h>

void get_serial_string(char *serial, size_t max_len)
{
    uint8_t unique_id[8];
    
    /* Read unique ID from flash */
    flash_get_unique_id(unique_id);
    
    /* Convert to hex string */
    if (max_len >= 17) {
        snprintf(serial, max_len, "%02X%02X%02X%02X%02X%02X%02X%02X",
                 unique_id[0], unique_id[1], unique_id[2], unique_id[3],
                 unique_id[4], unique_id[5], unique_id[6], unique_id[7]);
    } else {
        serial[0] = '\0';
    }
}

#elif defined(CONFIG_HWINFO)

/* Generic implementation using Zephyr's hwinfo if available */
#include <zephyr/drivers/hwinfo.h>

void get_serial_string(char *serial, size_t max_len)
{
    uint8_t device_id[16];
    ssize_t id_len;

    /* Try to get hardware unique ID */
    id_len = hwinfo_get_device_id(device_id, sizeof(device_id));

    if (id_len > 0) {
        /* Convert to hex string */
        size_t str_pos = 0;
        for (ssize_t i = 0; i < id_len && str_pos < (max_len - 2); i++) {
            str_pos += snprintf(&serial[str_pos], max_len - str_pos,
                               "%02X", device_id[i]);
        }
        serial[str_pos] = '\0';
    } else {
        /* Fallback to a default serial */
        if (max_len >= 5) {
            strncpy(serial, "0001", max_len - 1);
            serial[max_len - 1] = '\0';
        } else {
            serial[0] = '\0';
        }
        LOG_WRN("Could not get device ID, using default serial");
    }
}

size_t get_serial_raw(uint8_t *buffer, size_t max_len)
{
    return hwinfo_get_device_id(buffer, max_len);
}

#else

/* Fallback for platforms without hwinfo (e.g., native_sim) */
void get_serial_string(char *serial, size_t max_len)
{
    if (max_len >= 17) {
        strncpy(serial, "SIMULATION000001", max_len - 1);
        serial[max_len - 1] = '\0';
    } else if (max_len >= 5) {
        strncpy(serial, "SIM1", max_len - 1);
        serial[max_len - 1] = '\0';
    } else {
        serial[0] = '\0';
    }
}

size_t get_serial_raw(uint8_t *buffer, size_t max_len)
{
    /* Return a fixed ID for simulation */
    const uint8_t sim_id[] = {0x53, 0x49, 0x4D, 0x00, 0x00, 0x00, 0x00, 0x01};
    size_t len = MIN(max_len, sizeof(sim_id));
    memcpy(buffer, sim_id, len);
    return len;
}

#endif
