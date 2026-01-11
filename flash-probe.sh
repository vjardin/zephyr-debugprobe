#!/bin/bash
#
# Flash script for Raspberry Pi Debug Probe with ykushcmd power control
#
# Usage: ./flash-probe.sh [firmware.uf2] [mount_point]
#

# Configuration - adjust these as needed
YKUSH_PORT="${YKUSH_PORT:-2}"                    # ykushcmd port for debug probe
FIRMWARE="${1:-build/zephyr/zephyr.uf2}"         # Default firmware path
MOUNT_POINT="${2:-/media/${USER}/RPI-RP2}"       # Default DFU mount point
TIMEOUT="${TIMEOUT:-30}"                         # Timeout in seconds waiting for DFU mode

# Colors
R='\033[0;31m'
G='\033[0;32m'
Y='\033[1;33m'
N='\033[0m'

info()  { echo -e "${G}[INFO]${N} $1"; }
warn()  { echo -e "${Y}[WARN]${N} $1"; }
error() { echo -e "${R}[ERROR]${N} $1"; }

# Check if firmware file exists
if [[ ! -f "$FIRMWARE" ]]; then
    error "Firmware file not found: $FIRMWARE"
    echo "Usage: $0 [firmware.uf2] [mount_point]"
    exit 1
fi

info "Firmware: $FIRMWARE"
info "Mount point: $MOUNT_POINT"
info "ykushcmd port: $YKUSH_PORT"

# Function to power off the board
power_off() {
    info "Powering off debug probe..."
    /usr/bin/ykushcmd -d "$YKUSH_PORT" 2>/dev/null
    sleep 1
}

# Function to power on the board
power_on() {
    info "Powering on debug probe..."
    /usr/bin/ykushcmd -u "$YKUSH_PORT" 2>/dev/null
}

# Function to check if DFU mode is available
check_dfu_mode() {
    [[ -d "$MOUNT_POINT" ]] && mountpoint -q "$MOUNT_POINT" 2>/dev/null
}

# Function to wait for DFU mode with timeout
wait_for_dfu() {
    local elapsed=0
    local spinner=('|' '/' '-' '\')
    local spin_idx=0

    while [[ $elapsed -lt $TIMEOUT ]]; do
        if check_dfu_mode; then
            echo ""  # New line after spinner
            return 0
        fi

        # Show spinner
        printf "\r  Waiting for DFU mode... %s (%ds)" "${spinner[$spin_idx]}" "$elapsed"
        spin_idx=$(( (spin_idx + 1) % 4 ))

        sleep 1
        ((elapsed++))
    done

    echo ""  # New line after spinner
    return 1
}

# Main flashing loop
attempt=1
max_attempts=3

while [[ $attempt -le $max_attempts ]]; do
    info "=== Flash attempt $attempt of $max_attempts ==="

    # Power off the board
    power_off

    echo ""
    echo -e "${Y}>>> Please hold the BOOTSEL button on the debug probe <<<${N}"
    echo ""
    read -p "Press ENTER when ready to power on (while holding BOOTSEL)..."

    # Power on the board
    power_on

    # Wait for DFU mode
    info "Checking for DFU mode..."
    if wait_for_dfu; then
        info "DFU mode detected!"
        echo ""
        echo -e "${G}>>> You can release the BOOTSEL button now <<<${N}"
        echo ""
        info "Waiting 2 seconds before flashing..."
        sleep 2

        # Copy firmware
        info "Copying firmware to debug probe..."
        if cp "$FIRMWARE" "$MOUNT_POINT/"; then
            info "Firmware copied successfully!"

            # Wait for the board to reboot (mount point will disappear)
            info "Waiting for board to reboot..."
            sleep 2

            # Check if mount point disappeared (indicates successful flash)
            if ! check_dfu_mode; then
                echo ""
                info "=== Flash completed successfully! ==="
                echo ""
                exit 0
            else
                warn "Board may not have rebooted - please check manually"
                exit 0
            fi
        else
            error "Failed to copy firmware!"
            exit 1
        fi
    else
        warn "DFU mode not detected within ${TIMEOUT}s"

        if [[ $attempt -lt $max_attempts ]]; then
            echo ""
            warn "Let's try again. Make sure to:"
            echo "  1. Hold the BOOTSEL button BEFORE pressing ENTER"
            echo "  2. Keep holding until you see 'Checking for DFU mode'"
            echo ""
        fi
    fi

    ((attempt++))
done

error "Failed to enter DFU mode after $max_attempts attempts"
echo ""
echo "Troubleshooting tips:"
echo "  - Make sure the debug probe is connected to ykush port $YKUSH_PORT"
echo "  - The BOOTSEL button is the small button on the debug probe"
echo "  - Hold it BEFORE the board powers on"
echo "  - Check if mount point path is correct: $MOUNT_POINT"
echo ""
exit 1
