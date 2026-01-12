# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vincent Jardin <vjardin@free.fr>
#
# CMake module for PIO assembly compilation
#
# This module provides functions to compile .pio assembly files to C headers
# using the pioasm tool from the Raspberry Pi Pico SDK.
#

# Only relevant for RP2040
if(NOT CONFIG_SOC_RP2040)
    return()
endif()

# Find pioasm executable
# First check if it's already built in the build directory
# Then check system PATH
# Finally, build it from source

set(PIOASM_FOUND FALSE)

# Check for pre-built pioasm in common locations
find_program(PIOASM_EXECUTABLE pioasm
    HINTS
        ${CMAKE_BINARY_DIR}/pioasm
        ${CMAKE_BINARY_DIR}/pioasm-install/pioasm
        $ENV{PICO_SDK_PATH}/tools/pioasm/build
    PATHS
        /usr/bin
        /usr/local/bin
)

if(PIOASM_EXECUTABLE)
    set(PIOASM_FOUND TRUE)
    message(STATUS "Found pioasm: ${PIOASM_EXECUTABLE}")
else()
    # Need to build pioasm from source
    # Find pioasm source in the pico HAL module
    set(PIOASM_SOURCE_DIR "${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/tools/pioasm")

    if(EXISTS "${PIOASM_SOURCE_DIR}/CMakeLists.txt")
        message(STATUS "Building pioasm from source: ${PIOASM_SOURCE_DIR}")

        set(PIOASM_BINARY_DIR "${CMAKE_BINARY_DIR}/pioasm-build")
        set(PIOASM_INSTALL_DIR "${CMAKE_BINARY_DIR}/pioasm-install")

        # Get SDK version for pioasm version string
        set(PICO_SDK_VERSION_CMAKE "${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/pico_sdk_version.cmake")
        if(EXISTS "${PICO_SDK_VERSION_CMAKE}")
            include("${PICO_SDK_VERSION_CMAKE}")
            set(PIOASM_VERSION "${PICO_SDK_VERSION_STRING}")
        else()
            set(PIOASM_VERSION "2.0.0")  # Fallback version
        endif()

        # Find host compilers - pioasm is a build-time tool, not target code
        find_program(HOST_CXX_COMPILER NAMES g++ c++ clang++ PATHS /usr/bin /usr/local/bin)
        find_program(HOST_C_COMPILER NAMES gcc cc clang PATHS /usr/bin /usr/local/bin)
        if(NOT HOST_CXX_COMPILER)
            set(HOST_CXX_COMPILER "c++")
        endif()
        if(NOT HOST_C_COMPILER)
            set(HOST_C_COMPILER "cc")
        endif()

        # Configure and build pioasm as a host tool (explicitly use native compiler)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
                -S ${PIOASM_SOURCE_DIR}
                -B ${PIOASM_BINARY_DIR}
                -DCMAKE_INSTALL_PREFIX=${PIOASM_INSTALL_DIR}
                -DPIOASM_FLAT_INSTALL=1
                -DPIOASM_VERSION_STRING=${PIOASM_VERSION}
                -DCMAKE_C_COMPILER=${HOST_C_COMPILER}
                -DCMAKE_CXX_COMPILER=${HOST_CXX_COMPILER}
            RESULT_VARIABLE PIOASM_CMAKE_RESULT
            OUTPUT_QUIET
            ERROR_QUIET
        )

        if(PIOASM_CMAKE_RESULT EQUAL 0)
            # Build pioasm
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build ${PIOASM_BINARY_DIR} --target install
                RESULT_VARIABLE PIOASM_BUILD_RESULT
                OUTPUT_QUIET
                ERROR_QUIET
            )

            if(PIOASM_BUILD_RESULT EQUAL 0)
                set(PIOASM_EXECUTABLE "${PIOASM_INSTALL_DIR}/pioasm/pioasm")
                if(EXISTS ${PIOASM_EXECUTABLE})
                    set(PIOASM_FOUND TRUE)
                    message(STATUS "Built pioasm: ${PIOASM_EXECUTABLE}")
                endif()
            endif()
        endif()

        if(NOT PIOASM_FOUND)
            message(WARNING "Failed to build pioasm - PIO assembly will use pre-assembled fallback")
        endif()
    else()
        message(WARNING "pioasm source not found at ${PIOASM_SOURCE_DIR}")
    endif()
endif()

#
# Function: pioasm_generate_header
#
# Generate a C header from a PIO assembly file
#
# Arguments:
#   PIO_FILE    - Path to the .pio source file
#   OUTPUT_DIR  - Directory for generated header (defaults to CMAKE_CURRENT_BINARY_DIR)
#   OUTPUT_NAME - Name for generated header (defaults to <basename>.pio.h)
#
# Example:
#   pioasm_generate_header(
#       PIO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/pio/probe.pio
#       OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated
#   )
#
function(pioasm_generate_header)
    cmake_parse_arguments(PIOASM "" "PIO_FILE;OUTPUT_DIR;OUTPUT_NAME" "" ${ARGN})

    if(NOT PIOASM_PIO_FILE)
        message(FATAL_ERROR "pioasm_generate_header: PIO_FILE is required")
    endif()

    if(NOT EXISTS ${PIOASM_PIO_FILE})
        message(FATAL_ERROR "pioasm_generate_header: PIO file not found: ${PIOASM_PIO_FILE}")
    endif()

    # Default output directory
    if(NOT PIOASM_OUTPUT_DIR)
        set(PIOASM_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
    endif()

    # Get base name and default output name
    get_filename_component(PIO_BASENAME ${PIOASM_PIO_FILE} NAME_WE)
    if(NOT PIOASM_OUTPUT_NAME)
        set(PIOASM_OUTPUT_NAME "${PIO_BASENAME}.pio.h")
    endif()

    set(OUTPUT_HEADER "${PIOASM_OUTPUT_DIR}/${PIOASM_OUTPUT_NAME}")

    # Create output directory
    file(MAKE_DIRECTORY ${PIOASM_OUTPUT_DIR})

    if(PIOASM_FOUND)
        # Generate header using pioasm
        add_custom_command(
            OUTPUT ${OUTPUT_HEADER}
            COMMAND ${PIOASM_EXECUTABLE} -o c-sdk ${PIOASM_PIO_FILE} ${OUTPUT_HEADER}
            DEPENDS ${PIOASM_PIO_FILE}
            COMMENT "Generating PIO header: ${PIOASM_OUTPUT_NAME}"
            VERBATIM
        )

        # Add a target for the generated header
        add_custom_target(${PIO_BASENAME}_pio_header DEPENDS ${OUTPUT_HEADER})

        # Export variables to parent scope
        set(${PIO_BASENAME}_PIO_HEADER ${OUTPUT_HEADER} PARENT_SCOPE)
        set(${PIO_BASENAME}_PIO_INCLUDE_DIR ${PIOASM_OUTPUT_DIR} PARENT_SCOPE)
        set(PIOASM_GENERATED TRUE PARENT_SCOPE)

        message(STATUS "PIO header will be generated: ${OUTPUT_HEADER}")
    else()
        message(STATUS "pioasm not available - using pre-assembled PIO instructions")
        set(PIOASM_GENERATED FALSE PARENT_SCOPE)
    endif()
endfunction()
