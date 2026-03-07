#!/bin/bash

# Script to enable and start WiFi fallback service on RPi
# This ensures the WiFi hotspot starts automatically on boot

echo "========================================"
echo "Enabling WiFi Fallback Service"
echo "========================================"
echo ""

# Check if script is running as root
if [[ $EUID -ne 0 ]]; then
   echo "? This script must be run as root (sudo)"
   exit 1
fi

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "?? Working directory: $SCRIPT_DIR"
echo ""

# Step 1: Check if already installed
echo "Checking if wifi-fallback.service exists..."
if [[ -f /etc/systemd/system/wifi-fallback.service ]]; then
    echo "? Service file found"
else
    echo "? Service file not found, running install.sh first..."
    cd "$SCRIPT_DIR"
    bash install.sh
fi

echo ""

# Step 2: Enable the service
echo "Enabling wifi-fallback.service to start on boot..."
sudo systemctl enable wifi-fallback.service

echo ""

# Step 3: Start the service immediately
echo "Starting wifi-fallback.service now..."
sudo systemctl start wifi-fallback.service

echo ""

# Step 4: Verify status
echo "Checking service status..."
sudo systemctl status wifi-fallback.service --no-pager

echo ""

# Step 5: Show logs
echo "Last 10 log lines:"
echo "========================================"
sudo journalctl -u wifi-fallback.service -n 10 --no-pager 2>/dev/null || \
  tail -n 10 /var/log/rpi-wifi-fallback.log 2>/dev/null || \
  echo "No logs available yet"

echo ""
echo "========================================"
echo "? WiFi Fallback service enabled!"
echo "========================================"
echo ""
echo "Service will now:"
echo "  1. Start automatically on boot"
echo "  2. Try to connect to saved WiFi networks"
echo "  3. If no WiFi available, start hotspot:"
echo "     SSID: BFMCDemoCar"
echo "     Password: supersecurepassword"
echo "     IP: 192.168.50.1"
echo ""
echo "To check status anytime:"
echo "  sudo systemctl status wifi-fallback.service"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u wifi-fallback.service -f"
echo "  OR"
echo "  tail -f /var/log/rpi-wifi-fallback.log"
echo ""
