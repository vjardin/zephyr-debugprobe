# Debugprobe Zephyr Port

A port of the [Raspberry Pi Debug Probe](https://github.com/raspberrypi/debugprobe) firmware
(version cc88575eee0357ec49558f6f9636acddbb03fd7a) from FreeRTOS to Zephyr RTOS.

## Overview

The original debugprobe firmware runs on FreeRTOS with the Raspberry Pi Pico SDK. This port
provides equivalent functionality using the Zephyr RTOS, making it easier to integrate with other
Zephyr-based projects and supporting a wider range of hardware platforms.

### Features

- CMSIS-DAP v2 Debug Interface: SWD debugging for ARM Cortex-M targets
- USB-CDC UART Bridge: Serial port passthrough from target to host
- Multiple Threads: Concurrent handling of DAP, UART, and USB operations
- LED Indicators: Activity LEDs for TX, RX, and DAP operations
- Auto-baud Detection: Automatic UART baud rate detection (optional)

### Supported Boards

| Board                      | MCU    | SWD Method    | Notes                                      |
|----------------------------|--------|---------------|--------------------------------------------|
| rpi_debug_probe            | RP2040 | PIO           | Official Debug Probe with level shifters   |
| rpi_pico                   | RP2040 | PIO           | Best performance, hardware-accelerated SWD |
| rpi_pico/rp2040/w          | RP2040 | PIO           | WiFi variant, no LED (GPIO25 used by WiFi) |
| rpi_pico2/rp2350a/m33      | RP2350 | PIO           | Cortex-M33, 3 PIO blocks, 520KB RAM        |
| rpi_pico2/rp2350a/m33/w    | RP2350 | PIO           | WiFi variant, no LED                       |
| ek_ra4m2                   | RA4M2  | GPIO bit-bang | Renesas evaluation kit                     |
| native_sim                 | Host   | Stub          | Compile-time verification only             |
| native_sim/native/64       | Host   | Stub          | Compile-time verification only (64-bit)    |

Recommended: Official Raspberry Pi Debug Probe or Pico/Pico 2 for best SWD performance via PIO
hardware acceleration.

## FreeRTOS to Zephyr Mapping

This section documents the key conversions made from FreeRTOS to Zephyr APIs:

### Threading

| FreeRTOS               | Zephyr                   | Notes                      |
|------------------------|--------------------------|----------------------------|
| xTaskCreate()          | k_thread_create()        | Dynamic thread creation    |
| xTaskCreateStatic()    | K_THREAD_DEFINE()        | Static thread definition   |
| TaskHandle_t           | k_tid_t                  | Thread identifier          |
| vTaskDelay()           | k_sleep()                | Delay in ticks/ms          |
| vTaskDelayUntil()      | k_sleep() + timing calc  | Periodic delay             |
| taskYIELD()            | k_yield()                | Cooperative yield          |
| vTaskSuspend()         | k_thread_suspend()       | Suspend thread             |
| vTaskResume()          | k_thread_resume()        | Resume thread              |
| uxTaskPriorityGet()    | k_thread_priority_get()  | Get priority               |
| vTaskPrioritySet()     | k_thread_priority_set()  | Set priority               |
| tskIDLE_PRIORITY + n   | Priority number          | Lower = higher in Zephyr   |

### Synchronization Primitives

| FreeRTOS                    | Zephyr                   | Notes              |
|-----------------------------|--------------------------|--------------------|
| xSemaphoreCreateBinary()    | K_SEM_DEFINE(..., 0, 1)  | Binary semaphore   |
| xSemaphoreCreateCounting()  | K_SEM_DEFINE(..., n, max)| Counting semaphore |
| xSemaphoreCreateMutex()     | K_MUTEX_DEFINE()         | Mutex              |
| xSemaphoreGive()            | k_sem_give()             | Release semaphore  |
| xSemaphoreTake()            | k_sem_take()             | Acquire semaphore  |
| xQueueCreate()              | K_MSGQ_DEFINE()          | Message queue      |
| xQueueSend()                | k_msgq_put()             | Send to queue      |
| xQueueReceive()             | k_msgq_get()             | Receive from queue |

### Timing

| FreeRTOS                  | Zephyr                            | Notes                 |
|---------------------------|-----------------------------------|-----------------------|
| xTaskGetTickCount()       | k_uptime_get_32()                 | Current ticks (32-bit)|
| xTaskGetTickCountFromISR()| k_uptime_get_32()                 | Same in Zephyr        |
| pdMS_TO_TICKS(ms)         | K_MSEC(ms)                        | Milliseconds to ticks |
| portTICK_PERIOD_MS        | 1000/CONFIG_SYS_CLOCK_TICKS_PER_SEC | Tick period         |
| configTICK_RATE_HZ        | CONFIG_SYS_CLOCK_TICKS_PER_SEC    | Tick rate             |
| TickType_t                | int64_t or k_timeout_t            | Time type             |

### Memory Allocation

| FreeRTOS              | Zephyr               | Notes               |
|-----------------------|----------------------|---------------------|
| pvPortMalloc()        | k_malloc()           | Dynamic allocation  |
| vPortFree()           | k_free()             | Free memory         |
| configMINIMAL_STACK_SIZE | Kconfig stack sizes | Stack configuration |

### Interrupt Handling

| FreeRTOS              | Zephyr               | Notes                   |
|-----------------------|----------------------|-------------------------|
| portENTER_CRITICAL()  | irq_lock()           | Disable interrupts      |
| portEXIT_CRITICAL()   | irq_unlock()         | Enable interrupts       |
| portYIELD_FROM_ISR()  | Automatic in Zephyr  | Context switch from ISR |

## Building

### Prerequisites

- Zephyr SDK (>= 0.16.0)
- CMake (>= 3.20)
- Python 3
- west tool

### Install System Dependencies

```bash
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```

### Option 1: Using the West Manifest (Recommended)

This project includes a `west.yml` manifest that automatically fetches Zephyr and required
dependencies.

```bash
# Create workspace directory
mkdir -p ~/dev/debugprobe-workspace
cd ~/dev/debugprobe-workspace

# Create and activate Python virtual environment
python3 -m venv .venv
source .venv/bin/activate
pip install west

# Initialize workspace using this repo's manifest
west init -m https://github.com/vjardin/zephyr-debugprobe.git
# Or for local development:
# git clone https://github.com/vjardin/zephyr-debugprobe.git debugprobe
# west init -l debugprobe

# Fetch Zephyr and dependencies
west update

# Export Zephyr CMake package and install Python deps
west zephyr-export
pip install -r zephyr/scripts/requirements.txt

# Install Zephyr SDK
west sdk install --toolchains arm-zephyr-eabi --install-dir ~/toolchains/zephyr-sdk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/toolchains/zephyr-sdk
```

Build:

```bash
cd path_to_zephyr                      # buildX
source .venv/bin/activate              # buildX
export ZEPHYR_BASE=~/dev/zephyr/zephyr/zephyr # buildX

cd ~/dev/zephyr-debugprobe             # buildX
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr # buildX
export ZEPHYR_SDK_INSTALL_DIR=~/toolchains/zephyr-sdk      # buildX

# Build for Raspberry Pi Pico debug probe
west build -b rpi_debug_probe .        # buildX

# Build for Raspberry Pi Pico W (use separate build directory)
west build -b rpi_pico/rp2040/w -d build_pico_w debugprobe # buildX
```

Workspace structure:

```
debugprobe-workspace/
├── debugprobe/              # This project (manifest repo)
├── zephyr/                  # Zephyr RTOS
├── modules/
│   ├── hal/
│   │   ├── rpi_pico/        # RP2040/RP2350 HAL (Pico SDK)
│   │   └── cmsis_6/         # ARM CMSIS 6
│   └── ...
├── .venv/                   # Python virtual environment
└── .west/                   # West configuration
```

### Option 2: Using Existing Zephyr Installation

If you already have a Zephyr workspace to be shared with many applications, you can clone this
project and use it.

```bash
mkdir -p ~/dev/zephyr
cd ~/dev/zephyr

python3 -m venv .venv
source .venv/bin/activate

pip install west
west init -m https://github.com/zephyrproject-rtos/zephyr.git zephyr
west zephyr-export
cd zephyr
west update
pip install -r ../zephyr/scripts/requirements.txt
export ZEPHYR_BASE=~/dev/zephyr/zephyr/zephyr
west topdir

west sdk install --toolchains arm-zephyr-eabi --install-dir ~/toolchains/zephyr-sdk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/toolchains/zephyr-sdk
```

Build:

```bash
cd ~/dev/zephyr
source .venv/bin/activate
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/toolchains/zephyr-sdk
export ZEPHYR_BASE=~/dev/zephyr/zephyr

# Clone this project
git clone https://github.com/<your-username>/zephyr-debugprobe.git

# Build for Raspberry Pi Pico
cd zephyr-debugprobe
west build -b rpi_pico

# Build for Raspberry Pi Pico W
west build -b rpi_pico/rp2040/w -d build_pico_w
```

### Build for Other Boards

```bash
# For custom boards, create board overlays
west build -b <your_board> -- -DOVERLAY_CONFIG=boards/<your_board>.conf
```

### Build Notes

- Build Directory: Each target creates its own build directory (e.g., `build/rpi_pico`)
- Configuration: Use `west build -t menuconfig` to customize build options via the interactive menu

### Flash

The target board supports UDF.

```bash
# Using UF2 bootloader (hold BOOTSEL while connecting USB)
cp build/zephyr/zephyr.uf2 /media/<user>/RPI-RP2/
```

TBD: use openOCD/west flash

## Configuration

### Kconfig Options

| Option                          | Default | Description                    |
|---------------------------------|---------|--------------------------------|
| CONFIG_DEBUGPROBE               | y       | Enable debug probe functionality |
| CONFIG_DEBUGPROBE_CDC_UART      | y       | Enable CDC-UART bridge         |
| CONFIG_DEBUGPROBE_DAP           | y       | Enable CMSIS-DAP interface     |
| CONFIG_DEBUGPROBE_UART_BAUDRATE | 115200  | Default UART baud rate         |
| CONFIG_DEBUGPROBE_DAP_CLOCK_HZ  | 10000000 | SWD clock frequency (10 MHz)  |
| CONFIG_DEBUGPROBE_TX_LED        | y       | Enable TX activity LED         |
| CONFIG_DEBUGPROBE_RX_LED        | y       | Enable RX activity LED         |
| CONFIG_DEBUGPROBE_DAP_LED       | y       | Enable DAP activity LED        |
| CONFIG_DEBUGPROBE_AUTOBAUD      | y       | Enable auto-baud detection     |

### Pin Configuration

Default pin assignments for Raspberry Pi Pico (DEBUG_ON_PICO mode):

| Function | GPIO | Notes        |
|----------|------|--------------|
| SWCLK    | GP2  | SWD Clock    |
| SWDIO    | GP3  | SWD Data     |
| UART TX  | GP4  | Target UART TX |
| UART RX  | GP5  | Target UART RX |
| DAP LED  | GP25 | On-board LED |

For Debug Probe hardware, different pins are used (see `probe_config.h`).

## Shell Debug Commands

The firmware includes an interactive shell accessible via the USB CDC console (typically `/dev/ttyACM0`).
Connect at 115200 baud and press Enter to see the prompt.

### DAP Commands

| Command | Description |
|---------|-------------|
| `dap stats` | Show transfer statistics, clock frequency, connection status |
| `dap reset` | Reset transfer statistics counters |
| `dap clock` | Show system clock, SWCLK frequency, PIO status |
| `dap pins` | Display SWCLK, SWDIO, nRESET GPIO pin states |
| `dap trace on/off` | Enable/disable protocol tracing |

Example output:
```
debugprobe:~$ dap stats
DAP Status:
  Clock: 1000000 Hz
  Connected: no
  Running: no
  Trace: off
Transfer Statistics:
  Total: 0
  OK: 0
  WAIT: 0 (exhausted retries)
  FAULT: 0
  ERROR: 0
  Retries: 0
```

### PIO Commands (RP2040/RP2350 only)

| Command | Description |
|---------|-------------|
| `pio all` | Overview of both PIO blocks (instructions, state machines) |
| `pio status` | Detailed SWD state machine status (PIO0 SM0) |
| `pio debug` | Show FDEBUG register, clock divider, effective SWCLK |
| `pio reset` | Clear PIO debug flags |

Example output:
```
debugprobe:~$ pio all
RP2040 PIO Overview:
PIO0:
  Instructions: 11/32 (offset 21-31)
  SM0: enabled, PC=24, TX=0/4, RX=0/4
  FDEBUG: 0x01000000
PIO1:
  No state machines enabled
```

### UART Commands

| Command | Description |
|---------|-------------|
| `uart stats` | Show TX/RX byte counts, overruns, baud rate |
| `uart reset` | Reset UART statistics counters |

Example output:
```
debugprobe:~$ uart stats
UART Bridge Statistics:
  Status: running
  Baud rate: 115200
  TX bytes: 1234
  RX bytes: 5678
  TX overruns: 0
  RX overruns: 0
```

### Built-in Zephyr Commands

The shell also provides standard Zephyr commands:

| Command | Description |
|---------|-------------|
| `help` | List all available commands |
| `kernel threads` | Show running threads and stack usage |
| `kernel uptime` | Show system uptime |
| `device list` | List all device drivers |
| `gpio` | GPIO manipulation commands |

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        USB Host                             │
└──────────────────────────┬──────────────────────────────────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
    ┌──────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐
    │  CDC ACM    │ │  HID/Bulk   │ │   Status    │
    │  Interface  │ │  Interface  │ │  Interface  │
    └──────┬──────┘ └──────┬──────┘ └─────────────┘
           │               │
    ┌──────▼──────┐ ┌──────▼──────┐
    │ UART Thread │ │ DAP Thread  │
    │ (K_PRIO=5)  │ │ (K_PRIO=4)  │
    └──────┬──────┘ └──────┬──────┘
           │               │
    ┌──────▼──────┐ ┌──────▼──────┐
    │  Hardware   │ │  SWD/GPIO   │
    │    UART     │ │  or PIO     │
    └──────┬──────┘ └──────┬──────┘
           │               │
    ┌──────▼──────┐ ┌──────▼──────┐
    │   Target    │ │   Target    │
    │   Serial    │ │   Debug     │
    └─────────────┘ └─────────────┘
```

## Thread Model

Three main threads handle the different subsystems:

1. USB Thread (Priority 6): Handles USB device events
2. UART Thread (Priority 5): Bridges CDC ACM to hardware UART
3. DAP Thread (Priority 4): Processes CMSIS-DAP requests

Synchronization between USB and DAP uses semaphores:
- `dap_request_sem`: Signals when a new DAP request is available
- `dap_response_sem`: Signals when the response is ready

## Performance and PIO

### Why RP2040 Provides Best SWD Performance

The RP2040 (Raspberry Pi Pico) provides significantly better SWD debugging performance compared to
other MCUs due to its Programmable I/O (PIO) subsystem. While other platforms rely on software-based
"bit-bang" GPIO toggling, the RP2040 can offload the entire SWD protocol timing to dedicated PIO
state machines.

### What is PIO?

PIO (Programmable I/O) is a unique feature of the RP2040 that provides:

- Dedicated state machines: Four independent state machines per PIO block (8 total) that run in
  parallel with the CPU
- Deterministic timing: Cycle-accurate execution regardless of CPU load or interrupts
- High-speed operation: Can toggle pins at up to 125 MHz (system clock / 2)
- Protocol offloading: The CPU pushes data to FIFOs; PIO handles all bit-level timing

For SWD debugging, PIO enables:
- Sustained SWD clock rates up to 24 MHz (vs ~1-2 MHz with GPIO bit-bang)
- Zero CPU overhead during transfers - the CPU only feeds/reads FIFOs
- Precise timing that meets SWD protocol requirements without software intervention

### SWD Implementation Comparison

| Platform      | Method        | Typical Speed | CPU Usage                 |
|---------------|---------------|---------------|---------------------------|
| RP2040 (Pico) | PIO hardware  | Up to 24 MHz  | Minimal (FIFO ops only)   |
| Other MCUs    | GPIO bit-bang | 1-2 MHz       | High (constant toggling)  |

### Build-time PIO Assembly

This project uses pioasm to compile PIO assembly files (`.pio`) at build time:

- Source files: `src/pio/probe.pio` and `src/pio/probe_oen.pio`
- The CMake build system automatically builds pioasm from the Pico SDK if not found
- Falls back to pre-assembled instructions if pioasm cannot be built

The PIO programs handle:
- Clock generation (SWCLK)
- Data shifting (SWDIO read/write)
- Direction control (for level shifters via `probe_oen.pio`)
- Protocol timing (4 PIO cycles per SWD bit)

## Benchmark

Memory read throughput measured on nRF9151 target reading 64KB from RAM:

```bash
time pyocd cmd -t nrf91 -f <freq> -c "read32 0x20000000 16384" > /dev/null
```

### SWD Clock vs Throughput

| SWD Frequency | Time | Throughput | Status         |
|---------------|------|------------|----------------|
| 100 kHz       | 4.7s | 14 KB/s    | SWD-limited    |
| 1 MHz         | 2.1s | 31 KB/s    | SWD-limited    |
| 5 MHz         | 1.9s | 34 KB/s    | OK             |
| 10 MHz        | 1.9s | 35 KB/s    | OK             |
| 15 MHz        | 1.9s | 35 KB/s    | Max reliable   |
| 18 MHz        | -    | -          | ACK errors     |
| 20 MHz        | -    | -          | ACK errors     |

### Zephyr vs Original Debugprobe

| Implementation      | Time  | Throughput |
|---------------------|-------|------------|
| Zephyr port         | 1.85s | 35.4 KB/s  |
| Original debugprobe | 1.70s | 38.6 KB/s  |

Key findings:

- Maximum reliable SWD clock depends on target and wiring (~15 MHz in this test)
- Throughput plateaus above 5 MHz due to USB Full-Speed (12 Mbps) bottleneck
- Zephyr port achieves ~92% of original debugprobe throughput
- Both implementations are USB-limited at higher SWD frequencies

## Troubleshooting

### LED Indicators (Debug Probe Hardware)

The Debug Probe has 5 LEDs that indicate system status:

| LED | Color  | GPIO | Function                   |
|-----|--------|------|----------------------------|
| D1  | Red    | GP2  | USB connected / SWD error  |
| D2  | Green  | GP7  | UART RX activity           |
| D3  | Yellow | GP8  | UART TX activity           |
| D4  | Green  | GP15 | DAP connected              |
| D5  | Yellow | GP16 | DAP running/activity       |

### SWD LED Behavior

D1 Red LED - USB / Error Status:

| State                          | LED Behavior                  |
|--------------------------------|-------------------------------|
| USB connected                  | Steady ON                     |
| USB disconnected               | OFF                           |
| SWD wiring error (3+ failures) | Blinking (5 Hz)               |
| Error blinking continues       | Until 10s after last failure  |
| Successful SWD transfer        | Stops blinking, steady ON     |

D4 Green LED - DAP Connected:

| State                      | LED Behavior |
|----------------------------|--------------|
| Host debugger connected    | ON           |
| Host debugger disconnected | OFF          |

Controlled by host via CMSIS-DAP `DAP_HostStatus` command.

D5 Yellow LED - DAP Activity:

| State                          | LED Behavior   |
|--------------------------------|----------------|
| Host controls LED (rare)       | As commanded   |
| DAP commands being processed   | Blinking (4 Hz)|
| Idle (no activity for 500ms)   | OFF            |

Most debuggers (pyocd, OpenOCD) don't send activity status, so the firmware
automatically blinks during SWD operations.

### Diagnosing Connection Problems

Normal operation (target connected):

```
D1 Red:    ON (steady)     - USB connected
D4 Green:  ON              - DAP connected
D5 Yellow: Blinking        - Activity
```

Wiring problem or no target:

```
D1 Red:    BLINKING        - Error detected!
D4 Green:  ON              - Host thinks connected
D5 Yellow: Blinking        - Still attempting
```

USB not connected:

```
D1 Red:    OFF
D4 Green:  OFF
D5 Yellow: OFF
```

### Common Issues

Red LED blinking after pyocd command:

- Check SWD wiring (SWCLK, SWDIO, GND)
- Verify target is powered
- Try reducing SWD clock: `pyocd cmd -f 1000000 ...`
- Check `pio status` in shell for failure count

No response from target:

- Ensure SWCLK and SWDIO are not swapped
- Check for loose connections
- Verify target voltage compatibility (1.8V-3.3V)

Intermittent errors at high frequencies:

- Reduce SWD clock frequency
- Use shorter cables
- Check signal integrity

### Shell Diagnostics

Use the shell commands to diagnose issues:

```
debugprobe:~$ pio status
  Consecutive failures: 0 (threshold: 3)

debugprobe:~$ dap stats

debugprobe:~$ dap pins
```

## Differences from Original

1. RTOS: Zephyr instead of FreeRTOS
2. USB Stack: Zephyr USB device stack instead of TinyUSB (though TinyUSB can be used with Zephyr)
3. SDK: Zephyr HAL instead of Pico SDK (Pico SDK HAL can be accessed in Zephyr)
4. Build System: West/CMake instead of CMake only
5. PIO Access: Via Zephyr's PIO driver or direct hardware access

## License

MIT License - See individual source files for details.

Based on the original [debugprobe](https://github.com/raspberrypi/debugprobe) by Raspberry Pi.

## Contributing

Contributions are welcome! Please ensure your changes:
- Follow Zephyr coding style
- Include appropriate documentation
- Work with both GPIO and PIO modes where applicable
- Are tested on real hardware

## References

- [Zephyr Project Documentation](https://docs.zephyrproject.org/)
- [CMSIS-DAP Specification](https://arm-software.github.io/CMSIS_5/DAP/html/index.html)
- [Original Debug Probe Repository](https://github.com/raspberrypi/debugprobe)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/)
