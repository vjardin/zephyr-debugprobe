/* SPDX-License-Identifier: MIT
 *
 * Debugprobe Zephyr Port - CMSIS-DAP Configuration
 *
 * This header provides the hardware abstraction layer configuration
 * required by the CMSIS-DAP protocol stack. It follows the ARM
 * CMSIS-DAP specification for DAP_config.h.
 *
 * Reference: https://arm-software.github.io/CMSIS_5/DAP/html/DAP_Config_gr.html
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "probe_config.h"

/*
 * CPU Clock Configuration
 */

/** Processor clock of the Cortex-M MCU (Hz) */
#define CPU_CLOCK               125000000U

/** Maximum SWJ Clock Frequency (Hz) */
#define DAP_SWJ_CLOCK_MAX       15000000U

/** Default SWJ Clock Frequency (Hz) */
#define DAP_DEFAULT_PORT        1U      /* SWD mode */

/*
 * DAP Hardware Configuration
 */

/** Number of USB DAP packets to buffer for pipelining */
#ifndef DAP_PACKET_COUNT
#define DAP_PACKET_COUNT        4U
#endif

/** USB maximum packet size (bytes) */
#ifndef DAP_PACKET_SIZE
#define DAP_PACKET_SIZE         64U
#endif

/** Request packet buffer size for processing */
#define DAP_PACKET_BUFFER_SIZE  (DAP_PACKET_SIZE * DAP_PACKET_COUNT)

/*
 * DAP Feature Configuration
 */

/** SWD (Serial Wire Debug) support */
#define DAP_SWD                 1

/** JTAG support - conditional on hardware availability */
#if PROBE_JTAG_FULL_SUPPORT
#define DAP_JTAG                1
#else
#define DAP_JTAG                0
#endif

/** JTAG device count (max devices in chain) */
#define DAP_JTAG_DEV_CNT        8U

/** Atomic operations supported */
#define DAP_ATOMIC              0

/** Timestamp clock frequency (Hz), 0 = timestamps not supported */
#define DAP_TIMESTAMP_CLOCK     0U

/** Serial Wire Output (SWO) support */
#define DAP_SWO_UART            1
#define DAP_SWO_MANCHESTER      0

/** SWO buffer size */
#define DAP_SWO_BUFFER_SIZE     4096U

/** Streaming trace support */
#define DAP_SWO_STREAMING       0

/*
 * Transfer Configuration
 */

/** Default transfer retry count on WAIT response */
#define DAP_TRANSFER_RETRY      100U

/** Default number of extra idle cycles after each transfer */
#define DAP_TRANSFER_IDLE       0U

/** Default response match retry count */
#define DAP_TRANSFER_MATCH      0U

/*
 * SWD Protocol Configuration
 */

/** SWD turnaround period (cycles) */
#define DAP_SWD_TURNAROUND      1U

/** Generate SWD data parity on write */
#define DAP_SWD_DATA_PARITY     1

/*
 * Debug Unit Information
 */

/** Vendor name string */
#define DAP_VENDOR_STRING       "Raspberry Pi"

/** Product name string */
#define DAP_PRODUCT_STRING      "Debug Probe (Zephyr)"

/** Firmware version string */
#define DAP_FW_VERSION_STRING   "2.1.0"

/** Device vendor string */
#define DAP_DEVICE_VENDOR       ""

/** Device name string */
#define DAP_DEVICE_NAME         ""

/** Target board vendor */
#define DAP_TARGET_BOARD_VENDOR ""

/** Target board name */
#define DAP_TARGET_BOARD_NAME   ""

/*
 * CMSIS-DAP Command IDs (as per specification)
 */

/* General Commands */
#define ID_DAP_Info             0x00U
#define ID_DAP_HostStatus       0x01U
#define ID_DAP_Connect          0x02U
#define ID_DAP_Disconnect       0x03U
#define ID_DAP_TransferConfigure 0x04U
#define ID_DAP_Transfer         0x05U
#define ID_DAP_TransferBlock    0x06U
#define ID_DAP_TransferAbort    0x07U
#define ID_DAP_WriteABORT       0x08U
#define ID_DAP_Delay            0x09U
#define ID_DAP_ResetTarget      0x0AU

/* SWJ Commands */
#define ID_DAP_SWJ_Pins         0x10U
#define ID_DAP_SWJ_Clock        0x11U
#define ID_DAP_SWJ_Sequence     0x12U

/* SWD Commands */
#define ID_DAP_SWD_Configure    0x13U
#define ID_DAP_SWD_Sequence     0x1DU

/* SWO Commands */
#define ID_DAP_SWO_Transport    0x17U
#define ID_DAP_SWO_Mode         0x18U
#define ID_DAP_SWO_Baudrate     0x19U
#define ID_DAP_SWO_Control      0x1AU
#define ID_DAP_SWO_Status       0x1BU
#define ID_DAP_SWO_ExtendedStatus 0x1EU
#define ID_DAP_SWO_Data         0x1CU

/* JTAG Commands */
#define ID_DAP_JTAG_Sequence    0x14U
#define ID_DAP_JTAG_Configure   0x15U
#define ID_DAP_JTAG_IDCODE      0x16U

/* Queue Commands */
#define ID_DAP_QueueCommands    0x7EU
#define ID_DAP_ExecuteCommands  0x7FU

/* Vendor Commands (0x80-0x9F reserved) */
#define ID_DAP_Vendor0          0x80U
#define ID_DAP_Vendor31         0x9FU

/* Invalid/Error Response */
#define ID_DAP_Invalid          0xFFU

/*
 * DAP_Info IDs
 */
#define DAP_ID_VENDOR           0x01U
#define DAP_ID_PRODUCT          0x02U
#define DAP_ID_SER_NUM          0x03U
#define DAP_ID_FW_VER           0x04U
#define DAP_ID_DEVICE_VENDOR    0x05U
#define DAP_ID_DEVICE_NAME      0x06U
#define DAP_ID_TARGET_VENDOR    0x07U
#define DAP_ID_TARGET_NAME      0x08U
#define DAP_ID_BOARD_VENDOR     0x09U
#define DAP_ID_BOARD_NAME       0x0AU
#define DAP_ID_PRODUCT_FW_VER   0x0BU
#define DAP_ID_CAPABILITIES     0xF0U
#define DAP_ID_TIMESTAMP_CLOCK  0xF1U
#define DAP_ID_SWO_BUFFER_SIZE  0xFDU
#define DAP_ID_PACKET_COUNT     0xFEU
#define DAP_ID_PACKET_SIZE      0xFFU

/*
 * DAP Status Codes
 */
#define DAP_OK                  0x00U
#define DAP_ERROR               0xFFU

/*
 * DAP Port Modes
 */
#define DAP_PORT_DISABLED       0U
#define DAP_PORT_SWD            1U
#define DAP_PORT_JTAG           2U

/*
 * DAP Transfer Request Bits
 */
#define DAP_TRANSFER_APnDP      (1U << 0)  /* AP (1) or DP (0) access */
#define DAP_TRANSFER_RnW        (1U << 1)  /* Read (1) or Write (0) */
#define DAP_TRANSFER_A2         (1U << 2)  /* Address bit 2 */
#define DAP_TRANSFER_A3         (1U << 3)  /* Address bit 3 */
#define DAP_TRANSFER_MATCH_VALUE (1U << 4) /* Match value in transfer */
#define DAP_TRANSFER_MATCH_MASK (1U << 5)  /* Match mask follows */
#define DAP_TRANSFER_TIMESTAMP  (1U << 7)  /* Timestamp request */

/*
 * DAP Transfer Responses
 */
#define DAP_TRANSFER_OK         (1U << 0)
#define DAP_TRANSFER_WAIT       (1U << 1)
#define DAP_TRANSFER_FAULT      (1U << 2)
#define DAP_TRANSFER_ERROR      (1U << 3)
#define DAP_TRANSFER_MISMATCH   (1U << 4)

/*
 * SWO Transport Modes
 */
#define DAP_SWO_TRANSPORT_NONE  0U
#define DAP_SWO_TRANSPORT_CMD   1U
#define DAP_SWO_TRANSPORT_EP    2U

/*
 * SWO Modes
 */
#define DAP_SWO_MODE_OFF        0U
#define DAP_SWO_MODE_UART       1U
#define DAP_SWO_MODE_MANCHESTER 2U

/*
 * LED Status Types
 */
#define DAP_LED_DEBUGGER_CONNECTED  0U
#define DAP_LED_TARGET_RUNNING      1U

/*
 * Hardware Abstraction - Pin Access Macros
 *
 * These provide optimized access to SWD pins for bit-banging.
 * The actual implementation should use PIO where available.
 */

/** Get SWCLK pin state */
#define PIN_SWCLK_TCK_GET()     gpio_pin_get_raw(gpio_dev, PROBE_SWCLK_PIN)

/** Set SWCLK pin state */
#define PIN_SWCLK_TCK_SET(x)    gpio_pin_set_raw(gpio_dev, PROBE_SWCLK_PIN, x)

/** Get SWDIO pin state */
#define PIN_SWDIO_TMS_GET()     gpio_pin_get_raw(gpio_dev, PROBE_SWDIO_PIN)

/** Set SWDIO pin state */
#define PIN_SWDIO_TMS_SET(x)    gpio_pin_set_raw(gpio_dev, PROBE_SWDIO_PIN, x)

/** Set SWDIO direction (0=input, 1=output) */
#define PIN_SWDIO_OUT_ENABLE()  gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_OUTPUT)
#define PIN_SWDIO_OUT_DISABLE() gpio_pin_configure(gpio_dev, PROBE_SWDIO_PIN, GPIO_INPUT)

/** TDI pin access (JTAG) */
#if PROBE_TDI_PIN >= 0
#define PIN_TDI_GET()           gpio_pin_get_raw(gpio_dev, PROBE_TDI_PIN)
#define PIN_TDI_SET(x)          gpio_pin_set_raw(gpio_dev, PROBE_TDI_PIN, x)
#else
#define PIN_TDI_GET()           0
#define PIN_TDI_SET(x)          ((void)(x))
#endif

/** TDO pin access (JTAG) */
#if PROBE_TDO_PIN >= 0
#define PIN_TDO_GET()           gpio_pin_get_raw(gpio_dev, PROBE_TDO_PIN)
#else
#define PIN_TDO_GET()           0
#endif

/** nRESET pin access */
#if PROBE_RESET_PIN >= 0
#define PIN_nRESET_GET()        gpio_pin_get_raw(gpio_dev, PROBE_RESET_PIN)
#define PIN_nRESET_SET(x)       gpio_pin_set_raw(gpio_dev, PROBE_RESET_PIN, x)
#define PIN_nRESET_OUT_ENABLE() gpio_pin_configure(gpio_dev, PROBE_RESET_PIN, GPIO_OUTPUT_ACTIVE)
#else
#define PIN_nRESET_GET()        1
#define PIN_nRESET_SET(x)       ((void)(x))
#define PIN_nRESET_OUT_ENABLE() ((void)0)
#endif

/** nTRST pin access (JTAG) */
#if PROBE_TRST_PIN >= 0
#define PIN_nTRST_GET()         gpio_pin_get_raw(gpio_dev, PROBE_TRST_PIN)
#define PIN_nTRST_SET(x)        gpio_pin_set_raw(gpio_dev, PROBE_TRST_PIN, x)
#define PIN_nTRST_OUT_ENABLE()  gpio_pin_configure(gpio_dev, PROBE_TRST_PIN, GPIO_OUTPUT_ACTIVE)
#else
#define PIN_nTRST_GET()         1
#define PIN_nTRST_SET(x)        ((void)(x))
#define PIN_nTRST_OUT_ENABLE()  ((void)0)
#endif

/*
 * Target Voltage Detection
 */

/** Target running on target (0 = unknown) */
#define TARGET_DEVICE_VOLTAGE   3300U   /* 3.3V in mV */

/*
 * Capabilities Byte (DAP_ID_CAPABILITIES)
 *
 * Bit 0: SWD supported
 * Bit 1: JTAG supported
 * Bit 2: SWO UART supported
 * Bit 3: SWO Manchester supported
 * Bit 4: Atomic commands supported
 * Bit 5: Test domain timer supported
 * Bit 6: SWO streaming supported
 * Bit 7: UART communication port supported
 */
#define DAP_CAPABILITIES        ((DAP_SWD << 0) | \
                                 (DAP_JTAG << 1) | \
                                 (DAP_SWO_UART << 2) | \
                                 (DAP_SWO_MANCHESTER << 3) | \
                                 (DAP_ATOMIC << 4) | \
                                 (0 << 5) | \
                                 (DAP_SWO_STREAMING << 6) | \
                                 (1 << 7))

/*
 * Vendor Command IDs
 */
#define PROBE_VENDOR_VERSION        0x80U
#define PROBE_VENDOR_RESET_CONFIG   0x81U
#define PROBE_VENDOR_LED_CTRL       0x82U
#define PROBE_VENDOR_TARGET_VOLTAGE 0x83U
#define PROBE_VENDOR_RESET_TARGET   0x84U
#define PROBE_VENDOR_BOOT_MODE      0x85U
#define PROBE_VENDOR_GET_STATUS     0x86U
#define PROBE_VENDOR_SET_SYSCLK     0x87U

/*
 * Function Prototypes for Hardware Abstraction
 */

/**
 * @brief Setup debug port pins
 * Configure GPIO pins for SWD/JTAG operation
 */
void DAP_SETUP(void);

/**
 * @brief Connect to target
 * @param port Port mode (DAP_PORT_SWD or DAP_PORT_JTAG)
 * @return Actual port mode or DAP_PORT_DISABLED on error
 */
uint8_t DAP_PORT_CONNECT(uint8_t port);

/**
 * @brief Disconnect from target
 * Reset pins to safe state
 */
void DAP_PORT_DISCONNECT(void);

/**
 * @brief Set SWJ clock frequency
 * @param clock Requested clock frequency in Hz
 * @return Actual clock frequency set
 */
uint32_t DAP_SWJ_CLOCK(uint32_t clock);

/**
 * @brief Generate SWJ sequence
 * @param count Number of bits (0 = 256 bits)
 * @param data Sequence data (LSB first)
 */
void DAP_SWJ_Sequence(uint32_t count, const uint8_t *data);

/**
 * @brief Execute SWD sequence
 * @param info Sequence info (bit 7 = direction, bits 5:0 = length)
 * @param swdo Output data buffer
 * @param swdi Input data buffer (if reading)
 */
void DAP_SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi);

/**
 * @brief Set LED status
 * @param led LED type (DAP_LED_DEBUGGER_CONNECTED or DAP_LED_TARGET_RUNNING)
 * @param on LED state (true = on)
 */
void DAP_LED(uint8_t led, bool on);

/**
 * @brief Get timestamp
 * @return Current timestamp value
 */
uint32_t DAP_TIMESTAMP_GET(void);

/**
 * @brief Microsecond delay
 * @param usec Delay in microseconds
 */
void DAP_DELAY_US(uint32_t usec);

/*
 * External GPIO device reference (must be provided by implementation)
 */
extern const struct device *gpio_dev;

/*
 * Compatibility aliases for DAP_CMD_* naming convention
 * Maps to the standard ID_DAP_* names from CMSIS-DAP specification
 */
#define DAP_CMD_INFO            ID_DAP_Info
#define DAP_CMD_HOST_STATUS     ID_DAP_HostStatus
#define DAP_CMD_CONNECT         ID_DAP_Connect
#define DAP_CMD_DISCONNECT      ID_DAP_Disconnect
#define DAP_CMD_TRANSFER_CONFIGURE ID_DAP_TransferConfigure
#define DAP_CMD_TRANSFER        ID_DAP_Transfer
#define DAP_CMD_TRANSFER_BLOCK  ID_DAP_TransferBlock
#define DAP_CMD_TRANSFER_ABORT  ID_DAP_TransferAbort
#define DAP_CMD_WRITE_ABORT     ID_DAP_WriteABORT
#define DAP_CMD_DELAY           ID_DAP_Delay
#define DAP_CMD_RESET_TARGET    ID_DAP_ResetTarget
#define DAP_CMD_SWJ_PINS        ID_DAP_SWJ_Pins
#define DAP_CMD_SWJ_CLOCK       ID_DAP_SWJ_Clock
#define DAP_CMD_SWJ_SEQUENCE    ID_DAP_SWJ_Sequence
#define DAP_CMD_SWD_CONFIGURE   ID_DAP_SWD_Configure
#define DAP_CMD_SWD_SEQUENCE    ID_DAP_SWD_Sequence
#define DAP_CMD_JTAG_SEQUENCE   ID_DAP_JTAG_Sequence
#define DAP_CMD_JTAG_CONFIGURE  ID_DAP_JTAG_Configure
#define DAP_CMD_JTAG_IDCODE     ID_DAP_JTAG_IDCODE
#define DAP_CMD_SWO_TRANSPORT   ID_DAP_SWO_Transport
#define DAP_CMD_SWO_MODE        ID_DAP_SWO_Mode
#define DAP_CMD_SWO_BAUDRATE    ID_DAP_SWO_Baudrate
#define DAP_CMD_SWO_CONTROL     ID_DAP_SWO_Control
#define DAP_CMD_SWO_STATUS      ID_DAP_SWO_Status
#define DAP_CMD_SWO_EXT_STATUS  ID_DAP_SWO_ExtendedStatus
#define DAP_CMD_SWO_DATA        ID_DAP_SWO_Data
#define DAP_CMD_QUEUE_COMMANDS  ID_DAP_QueueCommands
#define DAP_CMD_EXECUTE_COMMANDS ID_DAP_ExecuteCommands
#define DAP_CMD_VENDOR_BASE     ID_DAP_Vendor0
