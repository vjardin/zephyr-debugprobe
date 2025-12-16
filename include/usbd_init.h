/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - USB Device Initialization Header
 */

#pragma once

#include <zephyr/usb/usbd.h>

/*
 * Initialize and configure the USB device.
 * Returns the USB device context on success, NULL on failure.
 */
struct usbd_context *debugprobe_usbd_init(usbd_msg_cb_t msg_cb);

/*
 * Get the USB device context.
 * Returns NULL if not initialized.
 */
struct usbd_context *debugprobe_usbd_get_context(void);
