/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - USB Device Initialization
 *
 * Uses the new Zephyr USB device stack (CONFIG_USB_DEVICE_STACK_NEXT)
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include "usbd_init.h"
#include "probe_config.h"

LOG_MODULE_REGISTER(debugprobe_usbd, CONFIG_LOG_DEFAULT_LEVEL);

/* Raspberry Pi Debug Probe USB identifiers */
#define DEBUGPROBE_USB_VID      0x2E8A
#define DEBUGPROBE_USB_PID      0x000C

/* USB power configuration */
#define DEBUGPROBE_MAX_POWER    125  /* 250mA in 2mA units */

/*
 * Instantiate USB device context using the default USB device controller
 */
USBD_DEVICE_DEFINE(debugprobe_usbd,
                   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   DEBUGPROBE_USB_VID, DEBUGPROBE_USB_PID);

/* String descriptors */
USBD_DESC_LANG_DEFINE(debugprobe_lang);
USBD_DESC_MANUFACTURER_DEFINE(debugprobe_mfr, "Raspberry Pi");
USBD_DESC_PRODUCT_DEFINE(debugprobe_product, "Debug Probe (Zephyr)");

#ifdef CONFIG_HWINFO
USBD_DESC_SERIAL_NUMBER_DEFINE(debugprobe_sn);
#endif

/* Configuration descriptor string */
USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "Full-Speed Configuration");

/* Configuration attributes - bus powered */
static const uint8_t cfg_attributes = 0;

/* Full-speed configuration */
USBD_CONFIGURATION_DEFINE(debugprobe_fs_config,
                          cfg_attributes,
                          DEBUGPROBE_MAX_POWER, &fs_cfg_desc);

/*
 * Set device class code triple based on registered classes
 */
static void fix_code_triple(struct usbd_context *uds_ctx,
                            const enum usbd_speed speed)
{
    /*
     * CDC ACM uses Interface Association Descriptor,
     * so we need to set the miscellaneous class code.
     */
    if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
        usbd_device_set_code_triple(uds_ctx, speed,
                                    USB_BCC_MISCELLANEOUS, 0x02, 0x01);
    } else {
        usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
    }
}

/*
 * Initialize and configure the USB device
 */
struct usbd_context *debugprobe_usbd_init(usbd_msg_cb_t msg_cb)
{
    int err;

    /* Add language descriptor */
    err = usbd_add_descriptor(&debugprobe_usbd, &debugprobe_lang);
    if (err) {
        LOG_ERR("Failed to add language descriptor: %d", err);
        return NULL;
    }

    /* Add manufacturer string */
    err = usbd_add_descriptor(&debugprobe_usbd, &debugprobe_mfr);
    if (err) {
        LOG_ERR("Failed to add manufacturer descriptor: %d", err);
        return NULL;
    }

    /* Add product string */
    err = usbd_add_descriptor(&debugprobe_usbd, &debugprobe_product);
    if (err) {
        LOG_ERR("Failed to add product descriptor: %d", err);
        return NULL;
    }

#ifdef CONFIG_HWINFO
    /* Add serial number from hardware info */
    err = usbd_add_descriptor(&debugprobe_usbd, &debugprobe_sn);
    if (err) {
        LOG_ERR("Failed to add serial number descriptor: %d", err);
        return NULL;
    }
#endif

    /* Add full-speed configuration */
    err = usbd_add_configuration(&debugprobe_usbd, USBD_SPEED_FS,
                                 &debugprobe_fs_config);
    if (err) {
        LOG_ERR("Failed to add FS configuration: %d", err);
        return NULL;
    }

    /* Register all available USB classes (CDC ACM, etc.) */
    err = usbd_register_all_classes(&debugprobe_usbd, USBD_SPEED_FS, 1, NULL);
    if (err) {
        LOG_ERR("Failed to register USB classes: %d", err);
        return NULL;
    }

    /* Set device class code triple */
    fix_code_triple(&debugprobe_usbd, USBD_SPEED_FS);

    /* Register message callback if provided */
    if (msg_cb != NULL) {
        err = usbd_msg_register_cb(&debugprobe_usbd, msg_cb);
        if (err) {
            LOG_ERR("Failed to register message callback: %d", err);
            return NULL;
        }
    }

    /* Initialize the USB device */
    err = usbd_init(&debugprobe_usbd);
    if (err) {
        LOG_ERR("Failed to initialize USB device: %d", err);
        return NULL;
    }

    LOG_INF("USB device initialized (VID=0x%04X, PID=0x%04X)",
            DEBUGPROBE_USB_VID, DEBUGPROBE_USB_PID);

    return &debugprobe_usbd;
}

/*
 * Get the USB device context
 */
struct usbd_context *debugprobe_usbd_get_context(void)
{
    return &debugprobe_usbd;
}
