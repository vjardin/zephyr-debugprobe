# SPDX-License-Identifier: MIT
# Debugprobe Zephyr Port - CMakeLists.txt
#
# This is a port of the Raspberry Pi Debug Probe firmware from FreeRTOS to Zephyr.
# Original: https://github.com/raspberrypi/debugprobe

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(debugprobe_zephyr VERSION 1.0.0 LANGUAGES C)

# Source files
target_sources(app PRIVATE
    src/main.c
    src/cdc_uart.c
    src/probe.c
    src/dap.c
    src/dap_vendor.c
    src/swo.c
    src/swdio.c
    src/get_serial.c
    src/watchdog.c
    src/debugprobe_usbd.c
    src/usbd_dap.c
)

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
