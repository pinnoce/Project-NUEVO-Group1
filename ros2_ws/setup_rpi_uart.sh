#!/usr/bin/env bash
# RPi one-time setup: enable hardware UART for Arduino bridge + harden against
# the sysrq-u read-only filesystem bug.
#
# Run as root or with sudo on the Raspberry Pi:
#   sudo bash setup_rpi_uart.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: run this script with sudo." >&2
    exit 1
fi

BOOT=/boot/firmware

# Determine the active cmdline.txt (RPi OS uses current/, Ubuntu plain /boot/firmware/)
if [ -f "$BOOT/current/cmdline.txt" ]; then
    CMDLINE="$BOOT/current/cmdline.txt"
else
    CMDLINE="$BOOT/cmdline.txt"
fi

CONFIG="$BOOT/config.txt"

echo "=== Remounting boot partition read-write ==="
mount -o remount,rw "$BOOT"

# ── Section 3.1: UART setup ──────────────────────────────────────────────────

echo "=== Enabling hardware UART in $CONFIG ==="
if grep -q "enable_uart=1" "$CONFIG"; then
    echo "  enable_uart=1 already present, skipping."
else
    echo "enable_uart=1" >> "$CONFIG"
    echo "  Added enable_uart=1"
fi

if grep -q "dtoverlay=uart0-pi5" "$CONFIG"; then
    echo "  dtoverlay=uart0-pi5 already present, skipping."
else
    echo "dtoverlay=uart0-pi5" >> "$CONFIG"
    echo "  Added dtoverlay=uart0-pi5"
fi

echo "=== Removing serial console entries from $CMDLINE ==="
cp "$CMDLINE" "${CMDLINE}.bak"
# Remove console=serial0,<baud> and console=ttyAMA0,<baud> (with or without baud)
sed -i 's/console=serial0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=ttyAMA0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=serial0 \?//g' "$CMDLINE"
sed -i 's/console=ttyAMA0 \?//g' "$CMDLINE"
echo "  Done. Backup saved to ${CMDLINE}.bak"
echo "  New cmdline: $(cat "$CMDLINE")"

# ── SysRq hardening ──────────────────────────────────────────────────────────

echo "=== Disabling SysRq ==="
echo "kernel.sysrq=0" > /etc/sysctl.d/99-disable-sysrq.conf
sysctl -w kernel.sysrq=0 > /dev/null
echo "  kernel.sysrq=$(cat /proc/sys/kernel/sysrq)"

# ── Summary ──────────────────────────────────────────────────────────────────

echo ""
echo "=== Done. Summary ==="
echo "  config.txt    : enable_uart=1, dtoverlay=uart0-pi5"
echo "  cmdline.txt   : serial console entries removed"
echo "  SysRq         : disabled (persists across reboots)"
echo ""
echo "Reboot to apply: sudo reboot"
echo "After reboot, verify with: ls -la /dev/ttyAMA0 && mount | grep 'on / '"
