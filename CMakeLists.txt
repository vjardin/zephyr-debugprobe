# SPDX-License-Identifier: MIT
# Debugprobe Zephyr Port - CMakeLists.txt
#
# This is a port of the Raspberry Pi Debug Probe firmware from FreeRTOS to Zephyr.
# Original: https://github.com/raspberrypi/debugprobe

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(debugprobe_zephyr VERSION 1.0.0 LANGUAGES C)

# Source files - always compiled
target_sources(app PRIVATE
    src/main.c
    src/get_serial.c
)

# USB device support (not available on native_sim)
if(CONFIG_USB_DEVICE_STACK_NEXT)
    target_sources(app PRIVATE
        src/debugprobe_usbd.c
        src/usbd_dap.c
    )
endif()

# DAP support
if(CONFIG_DEBUGPROBE_DAP)
    target_sources(app PRIVATE
        src/probe.c
        src/dap.c
        src/dap_vendor.c
        src/swo.c
        src/swdio.c
        src/watchdog.c
    )
endif()

# CDC UART support
if(CONFIG_DEBUGPROBE_CDC_UART)
    target_sources(app PRIVATE
        src/cdc_uart.c
    )
endif()

# PIO-based features for RP2040
if(CONFIG_SOC_RP2040)
    target_sources(app PRIVATE
        src/pio_swd.c
        src/autobaud.c
    )
endif()

# Include directories
target_include_directories(app PRIVATE
    include
    ${ZEPHYR_BASE}/include
)

# Compile definitions
target_compile_definitions(app PRIVATE
    DEBUGPROBE_VERSION="${PROJECT_VERSION}"
)

# Enable LTO for size optimization (optional)
if(CONFIG_LTO)
    set_property(TARGET app PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
