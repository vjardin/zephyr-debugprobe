/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - CMSIS-DAP USB Class (new stack)
 *
 * Implements a vendor-specific USB class with bulk endpoints for
 * CMSIS-DAP v2 protocol using the new Zephyr USB device stack.
 *
 * Uses ring buffer pattern matching original debugprobe for proper
 * support of pyocd's deferred transfer mechanism.
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "probe_config.h"
#include "DAP_config.h"  /* For ID_DAP_QueueCommands/ExecuteCommands */
#include "dap.h"

LOG_MODULE_REGISTER(usbd_dap, LOG_LEVEL_INF);

/* CMSIS-DAP interface string descriptor */
USBD_DESC_STRING_DEFINE(dap_iface_str, "CMSIS-DAP v2", USBD_DUT_STRING_INTERFACE);

/* DAP configuration */
#define DAP_BULK_EP_MPS     64      /* Full-speed bulk max packet size */

/* Class state flags */
#define DAP_CLASS_ENABLED       0

/*
 * Ring buffer structure (matching original debugprobe pattern)
 *
 * Uses separate read/write pointers with modulo indexing.
 * wasFull/wasEmpty flags handle edge cases for pipelining.
 */
typedef struct {
    uint8_t data[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
    volatile uint32_t wptr;      /* Write pointer (producer) */
    volatile uint32_t rptr;      /* Read pointer (consumer) */
    volatile bool wasEmpty;      /* Was empty when last checked */
    volatile bool wasFull;       /* Was full when last checked */
} dap_buffer_t;

/* Ring buffer index macros */
#define WR_IDX(x) ((x).wptr % DAP_PACKET_COUNT)
#define RD_IDX(x) ((x).rptr % DAP_PACKET_COUNT)
#define WR_SLOT(x) ((x).data[WR_IDX(x)])
#define RD_SLOT(x) ((x).data[RD_IDX(x)])

/* Ring buffer state checks */
static inline bool buffer_full(dap_buffer_t *buf)
{
    /* Full when next write would wrap to read position (correct for all values) */
    return ((buf->wptr + 1) % DAP_PACKET_COUNT == buf->rptr % DAP_PACKET_COUNT);
}

static inline bool buffer_empty(dap_buffer_t *buf)
{
    /* Empty when read has caught up to write */
    return (buf->wptr == buf->rptr);
}

/* Ring buffers for USB requests and responses */
static dap_buffer_t usb_request_buf;
static dap_buffer_t usb_response_buf;

/* Response length for current transfer */
static volatile uint32_t resp_len;

/* DAP processing thread for ring buffer handling */
#define USB_DAP_PROC_STACK_SIZE 2048
#define USB_DAP_PROC_PRIORITY   5
K_THREAD_STACK_DEFINE(usb_dap_proc_stack, USB_DAP_PROC_STACK_SIZE);
static struct k_thread usb_dap_proc_data;
static k_tid_t usb_dap_proc_id;

/* Semaphore to signal DAP processing thread */
K_SEM_DEFINE(usb_dap_proc_sem, 0, 10);

/* USB layer debug counters */
static volatile uint32_t usb_out_packets = 0;      /* Total OUT packets received */
static volatile uint32_t usb_out_zlp = 0;          /* Zero-length OUT packets */
static volatile uint32_t usb_in_packets = 0;       /* Total IN packets sent */
static volatile uint32_t usb_in_bytes = 0;         /* Total IN bytes sent */
static volatile uint32_t usb_errors = 0;           /* USB errors */
static volatile uint32_t usb_disabled_drops = 0;   /* Packets dropped while disabled */
static volatile uint8_t usb_last_cmd = 0;          /* Last command byte received */
static volatile uint8_t usb_last_resp = 0;         /* Last response byte sent */
static volatile uint32_t usb_last_req_len = 0;     /* Last request length */
static volatile uint32_t usb_last_resp_len = 0;    /* Last response length */

/* Packet sequence numbers for debugging */
static volatile uint32_t usb_out_seq = 0;          /* OUT packet sequence number */
static volatile uint32_t usb_in_seq = 0;           /* IN packet sequence number */

/* Ring buffer stats */
static volatile uint32_t rb_req_full_count = 0;    /* Request buffer was full */
static volatile uint32_t rb_resp_empty_count = 0;  /* Response buffer was empty */
static volatile uint32_t rb_max_pending = 0;       /* Max pending requests seen */

/* Detailed packet trace (ring buffer of last 64 events) */
#define TRACE_SIZE 64
struct trace_entry {
    uint32_t seq;           /* Sequence number */
    uint8_t dir;            /* 0=OUT, 1=IN */
    uint8_t cmd;            /* Command byte */
    uint8_t extra1;         /* Extra info (transfer count for 0x05/0x06) */
    uint8_t extra2;         /* Extra info (response count or ACK) */
    uint8_t req_w, req_r;   /* Request buffer state */
    uint8_t resp_w, resp_r; /* Response buffer state */
    uint16_t len;           /* Packet length */
};
static struct trace_entry packet_trace[TRACE_SIZE];
static volatile uint32_t trace_idx = 0;
static volatile bool trace_enabled = false;

/* Last DAP_Transfer details for tracing */
static volatile uint8_t last_transfer_req_count = 0;
static volatile uint8_t last_transfer_resp_count = 0;
static volatile uint8_t last_transfer_ack = 0;

static void add_trace(uint8_t dir, uint8_t cmd, uint16_t len)
{
    if (!trace_enabled) return;

    uint32_t idx = trace_idx % TRACE_SIZE;
    packet_trace[idx].seq = dir ? usb_in_seq : usb_out_seq;
    packet_trace[idx].dir = dir;
    packet_trace[idx].cmd = cmd;
    packet_trace[idx].req_w = usb_request_buf.wptr & 0xFF;
    packet_trace[idx].req_r = usb_request_buf.rptr & 0xFF;
    packet_trace[idx].resp_w = usb_response_buf.wptr & 0xFF;
    packet_trace[idx].resp_r = usb_response_buf.rptr & 0xFF;
    packet_trace[idx].len = len;

    /* For DAP_Transfer (0x05), capture request/response counts */
    if (cmd == 0x05) {
        if (dir == 0) {
            /* OUT: extra1 = transfer count from request */
            packet_trace[idx].extra1 = last_transfer_req_count;
            packet_trace[idx].extra2 = 0;
        } else {
            /* IN: extra1 = response count, extra2 = ACK */
            packet_trace[idx].extra1 = last_transfer_resp_count;
            packet_trace[idx].extra2 = last_transfer_ack;
        }
    } else {
        packet_trace[idx].extra1 = 0;
        packet_trace[idx].extra2 = 0;
    }

    trace_idx++;
}

/* Called from DAP processing to record Transfer details */
void usb_dap_trace_transfer(uint8_t req_count, uint8_t resp_count, uint8_t ack)
{
    last_transfer_req_count = req_count;
    last_transfer_resp_count = resp_count;
    last_transfer_ack = ack;
}

/* Get USB statistics for shell display */
void usb_dap_get_stats(uint32_t *out_pkts, uint32_t *out_zlp, uint32_t *in_pkts,
                       uint32_t *in_bytes, uint32_t *errors, uint32_t *disabled_drops,
                       uint8_t *last_cmd, uint8_t *last_resp,
                       uint32_t *last_req_len, uint32_t *last_resp_len)
{
    if (out_pkts) *out_pkts = usb_out_packets;
    if (out_zlp) *out_zlp = usb_out_zlp;
    if (in_pkts) *in_pkts = usb_in_packets;
    if (in_bytes) *in_bytes = usb_in_bytes;
    if (errors) *errors = usb_errors;
    if (disabled_drops) *disabled_drops = usb_disabled_drops;
    if (last_cmd) *last_cmd = usb_last_cmd;
    if (last_resp) *last_resp = usb_last_resp;
    if (last_req_len) *last_req_len = usb_last_req_len;
    if (last_resp_len) *last_resp_len = usb_last_resp_len;
}

/* Get ring buffer statistics for shell display */
void usb_dap_get_ringbuf_stats(uint32_t *req_wptr, uint32_t *req_rptr,
                               uint32_t *resp_wptr, uint32_t *resp_rptr,
                               uint32_t *req_full, uint32_t *resp_empty,
                               uint32_t *max_pending)
{
    if (req_wptr) *req_wptr = usb_request_buf.wptr;
    if (req_rptr) *req_rptr = usb_request_buf.rptr;
    if (resp_wptr) *resp_wptr = usb_response_buf.wptr;
    if (resp_rptr) *resp_rptr = usb_response_buf.rptr;
    if (req_full) *req_full = rb_req_full_count;
    if (resp_empty) *resp_empty = rb_resp_empty_count;
    if (max_pending) *max_pending = rb_max_pending;
}

/* Reset USB statistics */
void usb_dap_reset_stats(void)
{
    usb_out_packets = 0;
    usb_out_zlp = 0;
    usb_in_packets = 0;
    usb_in_bytes = 0;
    usb_errors = 0;
    usb_disabled_drops = 0;
    rb_req_full_count = 0;
    rb_resp_empty_count = 0;
    rb_max_pending = 0;
    usb_out_seq = 0;
    usb_in_seq = 0;
    trace_idx = 0;
}

/* Enable/disable packet trace */
void usb_dap_trace_enable(bool enable)
{
    trace_enabled = enable;
    if (enable) {
        trace_idx = 0;  /* Clear trace buffer when enabling */
    }
}

/* Dump packet trace to log */
void usb_dap_dump_trace(void)
{
    uint32_t count = (trace_idx < TRACE_SIZE) ? trace_idx : TRACE_SIZE;
    uint32_t start = (trace_idx < TRACE_SIZE) ? 0 : (trace_idx % TRACE_SIZE);

    LOG_INF("=== Packet Trace (last %u of %u events) ===", count, trace_idx);
    LOG_INF("Seq  Dir  Cmd   Len  Req(w/r)  Resp(w/r) Extra");

    for (uint32_t i = 0; i < count; i++) {
        uint32_t idx = (start + i) % TRACE_SIZE;
        struct trace_entry *e = &packet_trace[idx];

        if (e->cmd == 0x05 && (e->extra1 || e->extra2)) {
            /* DAP_Transfer with extra info */
            if (e->dir == 0) {
                LOG_INF("%3u  %s  0x%02x  %3u  %3u/%-3u   %3u/%-3u  req=%u",
                        e->seq, "OUT", e->cmd, e->len,
                        e->req_w, e->req_r, e->resp_w, e->resp_r,
                        e->extra1);
            } else {
                LOG_INF("%3u  %s  0x%02x  %3u  %3u/%-3u   %3u/%-3u  cnt=%u ack=%u",
                        e->seq, "IN ", e->cmd, e->len,
                        e->req_w, e->req_r, e->resp_w, e->resp_r,
                        e->extra1, e->extra2);
            }
        } else {
            LOG_INF("%3u  %s  0x%02x  %3u  %3u/%-3u   %3u/%-3u",
                    e->seq, e->dir ? "IN " : "OUT", e->cmd, e->len,
                    e->req_w, e->req_r, e->resp_w, e->resp_r);
        }
    }
    LOG_INF("=== End Trace ===");
}

/* Buffer pool for USB transfers - need DAP_PACKET_COUNT * 2 for OUT + IN */
NET_BUF_POOL_FIXED_DEFINE(dap_buf_pool, DAP_PACKET_COUNT * 2, DAP_PACKET_SIZE,
                          sizeof(struct udc_buf_info), NULL);

/* Class data structure */
struct dap_class_data {
    struct dap_class_desc *const desc;
    const struct usb_desc_header **const fs_desc;
    atomic_t state;
    struct usbd_class_data *c_data;  /* Back-reference for thread use */
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

/* Allocate a buffer for USB transfers */
static struct net_buf *dap_buf_alloc(uint8_t ep)
{
    struct net_buf *buf;
    struct udc_buf_info *bi;

    buf = net_buf_alloc(&dap_buf_pool, K_NO_WAIT);
    if (buf == NULL) {
        LOG_ERR("Failed to allocate buffer");
        return NULL;
    }

    bi = udc_get_buf_info(buf);
    memset(bi, 0, sizeof(struct udc_buf_info));
    bi->ep = ep;

    return buf;
}

/* Forward declaration */
static struct dap_class_data dap_class_data;

/*
 * DAP processing thread
 *
 * Processes commands from request ring buffer, writes responses to
 * response ring buffer. Signals USB layer when responses are ready.
 */
static void usb_dap_proc_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("DAP thread started");

    while (1) {
        /* Wait for work (request available or IN complete) */
        k_sem_take(&usb_dap_proc_sem, K_FOREVER);

        /* Process all pending requests - matching original debugprobe pattern */
        while (!buffer_empty(&usb_request_buf)) {
            struct usbd_class_data *c_data = dap_class_data.c_data;
            uint8_t in_ep;
            uint32_t pending;
            uint32_t n;
            uint8_t req_buf[DAP_PACKET_SIZE];
            uint8_t resp_buf[DAP_PACKET_SIZE];
            uint32_t rlen;

            if (c_data == NULL) {
                break;  /* Not initialized yet */
            }

            if (!atomic_test_bit(&dap_class_data.state, DAP_CLASS_ENABLED)) {
                break;  /* Class disabled */
            }

            in_ep = dap_get_bulk_in(c_data);

            /* Track max pending requests */
            pending = (usb_request_buf.wptr - usb_request_buf.rptr);
            if (pending > rb_max_pending) {
                rb_max_pending = pending;
            }

            /*
             * Atomic command support (matching original debugprobe):
             * Buffer QueueCommands but don't process them until a
             * non-QueueCommands packet is seen.
             */
            n = usb_request_buf.rptr;
            while (usb_request_buf.data[n % DAP_PACKET_COUNT][0] == ID_DAP_QueueCommands) {
                LOG_DBG("DAP queued cmd at %u", n);
                /* Change to ExecuteCommands so DAP layer processes it */
                usb_request_buf.data[n % DAP_PACKET_COUNT][0] = ID_DAP_ExecuteCommands;
                n++;
                /* Wait for more packets if we've caught up */
                while (n == usb_request_buf.wptr) {
                    LOG_DBG("DAP wait for more packets");
                    k_sem_take(&usb_dap_proc_sem, K_FOREVER);
                    /* Check if class was disabled while waiting */
                    if (!atomic_test_bit(&dap_class_data.state, DAP_CLASS_ENABLED)) {
                        break;
                    }
                }
                if (!atomic_test_bit(&dap_class_data.state, DAP_CLASS_ENABLED)) {
                    break;
                }
            }
            if (!atomic_test_bit(&dap_class_data.state, DAP_CLASS_ENABLED)) {
                break;
            }

            /*
             * Original debugprobe pattern:
             * 1. Copy request to LOCAL buffer (frees ring buffer slot)
             * 2. Increment rptr
             * 3. Handle wasFull (re-arm OUT endpoint)
             * 4. Process command from local buffer
             * 5. Handle response
             */

            /* Copy request to local buffer FIRST */
            memcpy(req_buf, RD_SLOT(usb_request_buf), DAP_PACKET_SIZE);
            usb_last_cmd = req_buf[0];

            /* Increment rptr - slot is now free for next packet */
            usb_request_buf.rptr++;

            /* If request buffer was full, re-arm OUT endpoint.
             * Use scheduler lock like original's vTaskSuspendAll */
            if (usb_request_buf.wasFull) {
                struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
                uint8_t out_ep = dap_get_bulk_out(c_data);
                struct net_buf *buf;

                k_sched_lock();
                usb_request_buf.wptr++;
                buf = dap_buf_alloc(out_ep);
                if (buf != NULL) {
                    int ret = usbd_ep_enqueue(c_data, buf);
                    if (ret) {
                        usbd_ep_buf_free(uds_ctx, buf);
                    }
                }
                usb_request_buf.wasFull = false;
                k_sched_unlock();
            }

            /* Now process command from local buffer */
            rlen = dap_process_request(req_buf, DAP_PACKET_SIZE,
                                       resp_buf, DAP_PACKET_SIZE);

            usb_last_resp = resp_buf[0];
            usb_last_resp_len = rlen;
            resp_len = rlen;

            /* Write response to response ring buffer.
             * Use scheduler lock like original's vTaskSuspendAll */
            k_sched_lock();

            if (buffer_empty(&usb_response_buf)) {
                /* Buffer was empty - copy response and send immediately */
                memcpy(WR_SLOT(usb_response_buf), resp_buf, rlen);
                usb_response_buf.wptr++;
                rb_resp_empty_count++;

                /* Arm IN endpoint with response */
                struct net_buf *buf = dap_buf_alloc(in_ep);
                if (buf != NULL) {
                    net_buf_add_mem(buf, RD_SLOT(usb_response_buf), rlen);
                    usb_in_bytes += rlen;

                    int ret = usbd_ep_enqueue(c_data, buf);
                    if (ret) {
                        struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
                        usbd_ep_buf_free(uds_ctx, buf);
                        usb_errors++;
                    }
                }
            } else {
                /* Buffer not empty - just queue response, IN callback will send */
                memcpy(WR_SLOT(usb_response_buf), resp_buf, rlen);
                usb_response_buf.wptr++;
                usb_response_buf.wasEmpty = false;
            }

            k_sched_unlock();
        }
    }
}

/*
 * USB class callback: endpoint request completed
 *
 * Ring buffer pattern:
 * - OUT complete: store in request buffer, re-arm OUT (unless full)
 * - IN complete: advance response read ptr, send next response (if available)
 */
static int dap_request_handler(struct usbd_class_data *const c_data,
                               struct net_buf *const buf, const int err)
{
    struct udc_buf_info *bi = udc_get_buf_info(buf);
    struct dap_class_data *data = usbd_class_get_private(c_data);
    struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
    uint8_t ep = bi->ep;
    uint8_t in_ep = dap_get_bulk_in(c_data);
    uint8_t out_ep = dap_get_bulk_out(c_data);

    if (!atomic_test_bit(&data->state, DAP_CLASS_ENABLED)) {
        usb_disabled_drops++;
        net_buf_unref(buf);
        return 0;
    }

    if (err != 0) {
        usb_errors++;
        net_buf_unref(buf);
        return err;
    }

    if (ep == in_ep) {
        /* IN transfer complete (response sent to host) */
        usb_in_seq++;
        usb_in_packets++;
        add_trace(1, usb_last_resp, usb_last_resp_len);

        /* Advance response read pointer */
        usb_response_buf.rptr++;

        /*
         * Original debugprobe pattern: only check wasEmpty, NOT buffer_empty.
         * wasEmpty=false means thread wrote when buffer wasn't empty,
         * so there's definitely more data to send.
         */
        if (!usb_response_buf.wasEmpty) {
            /* Reuse buffer for next response */
            memset(bi, 0, sizeof(struct udc_buf_info));
            bi->ep = in_ep;
            net_buf_reset(buf);

            size_t len = resp_len;
            net_buf_add_mem(buf, RD_SLOT(usb_response_buf), len);
            usb_in_bytes += len;

            /* Check if next send will empty the buffer */
            usb_response_buf.wasEmpty =
                ((usb_response_buf.rptr + 1) == usb_response_buf.wptr);

            int ret = usbd_ep_enqueue(c_data, buf);
            if (ret) {
                usbd_ep_buf_free(uds_ctx, buf);
                usb_errors++;
            }
        } else {
            /* No more responses - free buffer */
            net_buf_unref(buf);
        }

        /* Wake DAP thread (may have more work or buffer space now) */
        k_sem_give(&usb_dap_proc_sem);

    } else {
        /* OUT transfer complete (command received from host) */
        usb_out_seq++;
        usb_out_packets++;

        if (buf->len == 0) {
            /* Zero-length packet - re-arm */
            usb_out_zlp++;
            memset(bi, 0, sizeof(struct udc_buf_info));
            bi->ep = out_ep;
            net_buf_reset(buf);
            int ret = usbd_ep_enqueue(c_data, buf);
            if (ret) {
                usbd_ep_buf_free(uds_ctx, buf);
            }
        } else {
            /* Copy data to request ring buffer at current wptr slot */
            usb_last_req_len = buf->len;

            memcpy(WR_SLOT(usb_request_buf), buf->data,
                   MIN(buf->len, DAP_PACKET_SIZE));

            /* Record trace with command byte */
            add_trace(0, buf->data[0], buf->len);

            /*
             * Check if buffer can accept more BEFORE incrementing wptr.
             * This matches the original debugprobe pattern:
             * - If not full: increment wptr, re-arm into next slot
             * - If full: DON'T increment wptr, set wasFull (thread will increment)
             */
            if (!buffer_full(&usb_request_buf)) {
                usb_request_buf.wptr++;
                usb_request_buf.wasFull = false;

                /* Re-arm OUT endpoint into WR_SLOT (now the next slot) */
                memset(bi, 0, sizeof(struct udc_buf_info));
                bi->ep = out_ep;
                net_buf_reset(buf);

                int ret = usbd_ep_enqueue(c_data, buf);
                if (ret) {
                    usbd_ep_buf_free(uds_ctx, buf);
                }
            } else {
                /* Buffer full after this packet - don't increment wptr.
                 * DAP thread will increment wptr and re-arm after processing. */
                rb_req_full_count++;
                usb_request_buf.wasFull = true;
                net_buf_unref(buf);
            }

            /* Wake DAP thread to process command */
            k_sem_give(&usb_dap_proc_sem);
        }
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
    struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);

    LOG_INF("CMSIS-DAP class enabled");

    /* Note: DAP LEDs are controlled by host via DAP_HostStatus command */

    /* Reset ring buffers */
    usb_request_buf.wptr = 0;
    usb_request_buf.rptr = 0;
    usb_request_buf.wasEmpty = true;
    usb_request_buf.wasFull = false;

    usb_response_buf.wptr = 0;
    usb_response_buf.rptr = 0;
    usb_response_buf.wasEmpty = true;
    usb_response_buf.wasFull = false;

    if (!atomic_test_and_set_bit(&data->state, DAP_CLASS_ENABLED)) {
        uint8_t out_ep = dap_get_bulk_out(c_data);

        LOG_INF("Endpoints: OUT=0x%02x IN=0x%02x", out_ep,
                dap_get_bulk_in(c_data));
        LOG_INF("Ring buffer: %d slots of %d bytes", DAP_PACKET_COUNT,
                DAP_PACKET_SIZE);

        /* Store class data reference for thread use */
        data->c_data = c_data;

        /* Arm OUT endpoint to receive first command */
        struct net_buf *buf = dap_buf_alloc(out_ep);
        if (buf == NULL) {
            LOG_ERR("Failed to allocate initial buffer");
            return;
        }

        int ret = usbd_ep_enqueue(c_data, buf);
        if (ret) {
            LOG_ERR("Failed to enqueue initial buffer: %d", ret);
            usbd_ep_buf_free(uds_ctx, buf);
        } else {
            LOG_INF("DAP ready");
        }
    }
}

/*
 * USB class callback: class disabled
 */
static void dap_disable(struct usbd_class_data *const c_data)
{
    struct dap_class_data *data = usbd_class_get_private(c_data);

    atomic_clear_bit(&data->state, DAP_CLASS_ENABLED);
    LOG_INF("CMSIS-DAP class disabled");

    /* Wake DAP thread so it can exit its processing loop */
    k_sem_give(&usb_dap_proc_sem);
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
    struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
    int ret;

    /* Register CMSIS-DAP interface string descriptor */
    ret = usbd_add_descriptor(uds_ctx, &dap_iface_str);
    if (ret == 0) {
        /* Update interface descriptor with assigned string index */
        dap_desc.if0.iInterface = usbd_str_desc_get_idx(&dap_iface_str);
        LOG_INF("CMSIS-DAP interface string registered (idx=%d)",
                dap_desc.if0.iInterface);
    } else {
        LOG_WRN("Failed to add CMSIS-DAP interface string: %d", ret);
    }

    /* Start DAP processing thread */
    usb_dap_proc_id = k_thread_create(&usb_dap_proc_data, usb_dap_proc_stack,
                                       K_THREAD_STACK_SIZEOF(usb_dap_proc_stack),
                                       usb_dap_proc_entry, NULL, NULL, NULL,
                                       USB_DAP_PROC_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(usb_dap_proc_id, "usb_dap_proc");

    LOG_INF("CMSIS-DAP class initialized (ring buffer mode)");
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
    .c_data = NULL,
};

/* Register the class with USB device stack */
USBD_DEFINE_CLASS(cmsis_dap, &dap_class_api, &dap_class_data, NULL);
