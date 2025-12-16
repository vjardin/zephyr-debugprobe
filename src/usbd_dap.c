/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - CMSIS-DAP USB Class (new stack)
 *
 * Implements a vendor-specific USB class with bulk endpoints for
 * CMSIS-DAP v2 protocol using the new Zephyr USB device stack.
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "probe_config.h"
#include "dap.h"

LOG_MODULE_REGISTER(usbd_dap, CONFIG_LOG_DEFAULT_LEVEL);

/* DAP processing function from dap.c */
extern uint32_t dap_process_request(const uint8_t *request, uint32_t request_len,
                                    uint8_t *response, uint32_t response_len);

/* DAP configuration */
#define DAP_BULK_EP_MPS     64      /* Full-speed bulk max packet size */

/* Class state flags */
#define DAP_CLASS_ENABLED       0
#define DAP_CLASS_OUT_ENGAGED   1
#define DAP_CLASS_IN_ENGAGED    2

/* Internal buffer for DAP response processing */
static uint8_t dap_response_buf[DAP_PACKET_SIZE];

/* Multi-packet circular buffer for pipelining */
struct dap_packet {
    uint8_t data[DAP_PACKET_SIZE];
    uint32_t len;
    bool valid;
};

static struct dap_packet request_queue[DAP_PACKET_COUNT];
static struct dap_packet response_queue[DAP_PACKET_COUNT];
static volatile uint8_t req_write_idx;
static volatile uint8_t req_read_idx;
static volatile uint8_t resp_write_idx;
static volatile uint8_t resp_read_idx;

/* Semaphore for DAP processing thread */
static K_SEM_DEFINE(dap_process_sem, 0, 1);

/* Class data structure */
struct dap_class_data {
    struct dap_class_desc *const desc;
    const struct usb_desc_header **const fs_desc;
    atomic_t state;
};

/* CMSIS-DAP interface descriptor structure */
struct dap_class_desc {
    struct usb_if_descriptor if0;           /* Vendor interface */
    struct usb_ep_descriptor if0_out_ep;    /* Bulk OUT endpoint */
    struct usb_ep_descriptor if0_in_ep;     /* Bulk IN endpoint */
    struct usb_desc_header nil_desc;        /* Terminator */
};

/* Descriptor instance */
static struct dap_class_desc dap_desc = {
    /* CMSIS-DAP vendor interface */
    .if0 = {
        .bLength = sizeof(struct usb_if_descriptor),
        .bDescriptorType = USB_DESC_INTERFACE,
        .bInterfaceNumber = 0,              /* Will be assigned by stack */
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_BCC_VENDOR,  /* Vendor specific */
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0,                    /* No string descriptor */
    },

    /* Bulk OUT endpoint (host to device) */
    .if0_out_ep = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x01,           /* OUT EP1 */
        .bmAttributes = USB_EP_TYPE_BULK,
        .wMaxPacketSize = sys_cpu_to_le16(DAP_BULK_EP_MPS),
        .bInterval = 0x00,
    },

    /* Bulk IN endpoint (device to host) */
    .if0_in_ep = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x81,           /* IN EP1 */
        .bmAttributes = USB_EP_TYPE_BULK,
        .wMaxPacketSize = sys_cpu_to_le16(DAP_BULK_EP_MPS),
        .bInterval = 0x00,
    },

    /* Terminator */
    .nil_desc = {
        .bLength = 0,
        .bDescriptorType = 0,
    },
};

/* Full-speed descriptor list */
static const struct usb_desc_header *dap_fs_desc[] = {
    (struct usb_desc_header *)&dap_desc.if0,
    (struct usb_desc_header *)&dap_desc.if0_out_ep,
    (struct usb_desc_header *)&dap_desc.if0_in_ep,
    (struct usb_desc_header *)&dap_desc.nil_desc,
};

/* Get bulk OUT endpoint address */
static uint8_t dap_get_bulk_out(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);
    return data->desc->if0_out_ep.bEndpointAddress;
}

/* Get bulk IN endpoint address */
static uint8_t dap_get_bulk_in(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);
    return data->desc->if0_in_ep.bEndpointAddress;
}

/* Circular buffer helpers */
static inline uint8_t buf_next_idx(uint8_t idx)
{
    return (idx + 1) % DAP_PACKET_COUNT;
}

static inline bool request_queue_full(void)
{
    return buf_next_idx(req_write_idx) == req_read_idx;
}

static inline bool request_queue_empty(void)
{
    return req_write_idx == req_read_idx;
}

static inline bool response_queue_empty(void)
{
    return resp_write_idx == resp_read_idx;
}

/* Submit a bulk OUT transfer to receive data from host */
static int dap_submit_bulk_out(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);
    struct net_buf *buf;
    int err;

    if (!atomic_test_bit(&data->state, DAP_CLASS_ENABLED)) {
        return -EPERM;
    }

    if (atomic_test_and_set_bit(&data->state, DAP_CLASS_OUT_ENGAGED)) {
        return -EBUSY;
    }

    if (request_queue_full()) {
        atomic_clear_bit(&data->state, DAP_CLASS_OUT_ENGAGED);
        return -ENOBUFS;
    }

    buf = usbd_ep_buf_alloc(c_data, dap_get_bulk_out(c_data), DAP_PACKET_SIZE);
    if (buf == NULL) {
        atomic_clear_bit(&data->state, DAP_CLASS_OUT_ENGAGED);
        LOG_ERR("Failed to allocate OUT buffer");
        return -ENOMEM;
    }

    err = usbd_ep_enqueue(c_data, buf);
    if (err) {
        net_buf_unref(buf);
        atomic_clear_bit(&data->state, DAP_CLASS_OUT_ENGAGED);
        LOG_ERR("Failed to enqueue OUT buffer: %d", err);
    }

    return err;
}

/* Submit a bulk IN transfer to send data to host */
static int dap_submit_bulk_in(struct usbd_class_data *const c_data,
                              const uint8_t *response, uint32_t len)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);
    struct net_buf *buf;
    int err;

    if (!atomic_test_bit(&data->state, DAP_CLASS_ENABLED)) {
        return -EPERM;
    }

    if (atomic_test_and_set_bit(&data->state, DAP_CLASS_IN_ENGAGED)) {
        return -EBUSY;
    }

    buf = usbd_ep_buf_alloc(c_data, dap_get_bulk_in(c_data), len);
    if (buf == NULL) {
        atomic_clear_bit(&data->state, DAP_CLASS_IN_ENGAGED);
        LOG_ERR("Failed to allocate IN buffer");
        return -ENOMEM;
    }

    net_buf_add_mem(buf, response, len);

    err = usbd_ep_enqueue(c_data, buf);
    if (err) {
        net_buf_unref(buf);
        atomic_clear_bit(&data->state, DAP_CLASS_IN_ENGAGED);
        LOG_ERR("Failed to enqueue IN buffer: %d", err);
    }

    return err;
}

/* Try to send next response from queue */
static void dap_try_send_response(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);

    if (!atomic_test_bit(&data->state, DAP_CLASS_ENABLED)) {
        return;
    }

    if (atomic_test_bit(&data->state, DAP_CLASS_IN_ENGAGED)) {
        return;  /* Already sending */
    }

    if (response_queue_empty()) {
        return;  /* Nothing to send */
    }

    struct dap_packet *resp = &response_queue[resp_read_idx];
    if (!resp->valid) {
        resp_read_idx = buf_next_idx(resp_read_idx);
        return;
    }

    if (dap_submit_bulk_in(c_data, resp->data, resp->len) == 0) {
        resp->valid = false;
        resp_read_idx = buf_next_idx(resp_read_idx);
    }
}

/* Process pending DAP requests - called from workqueue context */
static void dap_process_pending(struct usbd_class_data *const c_data)
{
    while (!request_queue_empty()) {
        struct dap_packet *req = &request_queue[req_read_idx];

        if (!req->valid) {
            req_read_idx = buf_next_idx(req_read_idx);
            continue;
        }

        /* Check if we have space for response */
        if (buf_next_idx(resp_write_idx) == resp_read_idx) {
            break;  /* Response queue full */
        }

        /* Process DAP command */
        uint32_t resp_len = dap_process_request(req->data, req->len,
                                                dap_response_buf, DAP_PACKET_SIZE);

        req->valid = false;
        req_read_idx = buf_next_idx(req_read_idx);

        if (resp_len > 0) {
            struct dap_packet *resp = &response_queue[resp_write_idx];
            memcpy(resp->data, dap_response_buf, resp_len);
            resp->len = resp_len;
            resp->valid = true;
            resp_write_idx = buf_next_idx(resp_write_idx);
        }
    }

    /* Try to send any pending responses */
    dap_try_send_response(c_data);

    /* Re-arm the OUT endpoint if there's room */
    dap_submit_bulk_out(c_data);
}

/*
 * USB class callback: endpoint request completed
 */
static int dap_request_handler(struct usbd_class_data *const c_data,
                               struct net_buf *const buf, const int err)
{
    struct udc_buf_info *bi = (struct udc_buf_info *)net_buf_user_data(buf);
    struct dap_class_data *data = usbd_class_get_private(c_data);
    const uint8_t ep = bi->ep;
    const size_t len = buf->len;

    if (ep == dap_get_bulk_out(c_data)) {
        /* OUT transfer complete - data received from host */
        atomic_clear_bit(&data->state, DAP_CLASS_OUT_ENGAGED);

        if (err == 0 && len > 0) {
            /* Queue the request */
            struct dap_packet *req = &request_queue[req_write_idx];
            memcpy(req->data, buf->data, MIN(len, DAP_PACKET_SIZE));
            req->len = len;
            req->valid = true;
            req_write_idx = buf_next_idx(req_write_idx);

            LOG_DBG("DAP RX: %u bytes, cmd=0x%02x", (unsigned)len, buf->data[0]);
        }
    } else if (ep == dap_get_bulk_in(c_data)) {
        /* IN transfer complete - data sent to host */
        atomic_clear_bit(&data->state, DAP_CLASS_IN_ENGAGED);

        if (err == 0) {
            LOG_DBG("DAP TX: %u bytes", (unsigned)len);
        }
    }

    net_buf_unref(buf);

    if (err == -ECONNABORTED) {
        LOG_DBG("Transfer ep 0x%02x cancelled", ep);
    } else if (err != 0) {
        LOG_ERR("Transfer ep 0x%02x failed: %d", ep, err);
        return err;
    }

    /* Process any pending work */
    if (atomic_test_bit(&data->state, DAP_CLASS_ENABLED)) {
        dap_process_pending(c_data);
    }

    return 0;
}

/*
 * USB class callback: interface/alternate setting changed
 */
static void dap_update(struct usbd_class_data *c_data,
                       uint8_t iface, uint8_t alternate)
{
    LOG_DBG("Interface %u alternate %u", iface, alternate);
}

/*
 * USB class callback: get descriptor
 */
static void *dap_get_desc(struct usbd_class_data *const c_data,
                          const enum usbd_speed speed)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);
    return data->fs_desc;  /* Only full-speed supported on RP2040 */
}

/*
 * USB class callback: class enabled (configuration selected)
 */
static void dap_enable(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);

    LOG_INF("CMSIS-DAP class enabled");

    /* Clear queues */
    memset(request_queue, 0, sizeof(request_queue));
    memset(response_queue, 0, sizeof(response_queue));
    req_write_idx = req_read_idx = 0;
    resp_write_idx = resp_read_idx = 0;

    atomic_set_bit(&data->state, DAP_CLASS_ENABLED);

    /* Start receiving data */
    dap_submit_bulk_out(c_data);
}

/*
 * USB class callback: class disabled
 */
static void dap_disable(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);

    atomic_clear_bit(&data->state, DAP_CLASS_ENABLED);
    LOG_INF("CMSIS-DAP class disabled");
}

/*
 * USB class callback: device suspended
 */
static void dap_suspended(struct usbd_class_data *const c_data)
{
    LOG_DBG("CMSIS-DAP suspended");
}

/*
 * USB class callback: device resumed
 */
static void dap_resumed(struct usbd_class_data *const c_data)
{
    LOG_DBG("CMSIS-DAP resumed");
}

/*
 * USB class callback: class initialization
 */
static int dap_class_init(struct usbd_class_data *c_data)
{
    LOG_INF("CMSIS-DAP class initialized");
    return 0;
}

/* Class API structure */
static struct usbd_class_api dap_class_api = {
    .update = dap_update,
    .request = dap_request_handler,
    .get_desc = dap_get_desc,
    .enable = dap_enable,
    .disable = dap_disable,
    .suspended = dap_suspended,
    .resumed = dap_resumed,
    .init = dap_class_init,
};

/* Class data instance */
static struct dap_class_data dap_class_data = {
    .desc = &dap_desc,
    .fs_desc = dap_fs_desc,
    .state = ATOMIC_INIT(0),
};

/* Register the class with USB device stack */
USBD_DEFINE_CLASS(cmsis_dap, &dap_class_api, &dap_class_data, NULL);
