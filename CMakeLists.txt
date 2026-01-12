# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
#
# Debugprobe Zephyr Port - CMakeLists.txt
#
# This is a port of the Raspberry Pi Debug Probe firmware from FreeRTOS to Zephyr.
# Original: https://github.com/raspberrypi/debugprobe

cmake_minimum_required(VERSION 3.20.0)

# Add custom devicetree bindings (for debugprobe-swd)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(debugprobe_zephyr VERSION 1.0.0 LANGUAGES C)

# Source files - always compiled
target_sources(app PRIVATE
    src/main.c
    src/get_serial.c
    src/led.c
)

# USB device support
if(CONFIG_USB_DEVICE_STACK_NEXT)
    target_sources(app PRIVATE
        src/debugprobe_usbd.c
    )
    # USBD DAP requires both USB and DAP to be enabled
    if(CONFIG_DEBUGPROBE_DAP)
        target_sources(app PRIVATE
            src/usbd_dap.c
        )
    endif()
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

    # PIO assembly compilation
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/pioasm.cmake)

    # Generate PIO headers from .pio source files
    set(PIO_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/pio)
    file(MAKE_DIRECTORY ${PIO_GENERATED_DIR})

    pioasm_generate_header(
        PIO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/pio/probe.pio
        OUTPUT_DIR ${PIO_GENERATED_DIR}
    )

    pioasm_generate_header(
        PIO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/pio/probe_oen.pio
        OUTPUT_DIR ${PIO_GENERATED_DIR}
    )

    # Add generated headers as dependencies if pioasm is available
    if(PIOASM_GENERATED)
        add_dependencies(app probe_pio_header probe_oen_pio_header)
        target_compile_definitions(app PRIVATE
            PIO_GENERATED_HEADERS=1
        )
    endif()

    # Add generated header directory to include path
    target_include_directories(app PRIVATE
        ${PIO_GENERATED_DIR}
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
