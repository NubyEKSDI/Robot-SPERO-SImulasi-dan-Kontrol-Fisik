#!/bin/bash
# Script untuk install dependencies RP-Lidar A1M8

echo "=== INSTALLING RP-LIDAR A1M8 DEPENDENCIES ==="

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install Python pip if not installed
if ! command -v pip3 &> /dev/null; then
    echo "Installing Python pip..."
    sudo apt-get install -y python3-pip
fi

# Install required Python packages
echo "Installing Python dependencies..."
pip3 install rplidar-roboticia
pip3 install pyserial

# Add user to dialout group for USB access
echo "Adding user to dialout group..."
sudo usermod -a -G dialout $USER

# Set USB permissions
echo "Setting USB permissions..."
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || echo "Note: /dev/ttyUSB0 not found yet (normal if LiDAR not connected)"

# Install additional useful packages
echo "Installing additional packages..."
sudo apt-get install -y usbutils
sudo apt-get install -y udev

# Create udev rule for consistent device naming
echo "Creating udev rule for LiDAR..."
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null <<EOF
# RP-Lidar A1M8 udev rule
SUBSYSTEM=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
EOF

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "=== INSTALLATION COMPLETE ==="
echo ""
echo "Next steps:"
echo "1. Connect RP-Lidar A1M8 to USB port"
echo "2. Logout and login again (or reboot) for group changes to take effect"
echo "3. Run: python3 test_lidar.py"
echo "4. If successful, run: python3 test_robot.py"
echo ""
echo "Troubleshooting:"
echo "- If permission denied: sudo chmod 666 /dev/ttyUSB0"
echo "- If device not found: ls /dev/ttyUSB*"
echo "- Check USB connection: lsusb | grep 10c4" 