#!/bin/bash

# Script untuk memulai test_robot.py di Raspberry Pi
# Usage: ./start_test_robot_rpi.sh
# 
# Script ini harus dijalankan di Raspberry Pi yang terhubung dengan lidar A1M8

echo "=== Starting test_robot.py on Raspberry Pi ==="
echo "Prerequisites:"
echo "1. A1M8 lidar terhubung ke /dev/ttyUSB0"
echo "2. Python dependencies terinstall (rplidar-roboticia, paho-mqtt)"
echo "3. MQTT broker (codex.petra.ac.id) dapat diakses"
echo ""

# Check if lidar device exists
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "ERROR: Lidar device /dev/ttyUSB0 not found!"
    echo "Please check lidar connection and try again."
    exit 1
fi

# Check device permissions
if [ ! -r "/dev/ttyUSB0" ]; then
    echo "WARNING: Cannot read /dev/ttyUSB0"
    echo "Trying to fix permissions..."
    sudo chmod 666 /dev/ttyUSB0
fi

# Check MQTT connectivity
echo "Checking MQTT connectivity..."
if ! ping -c 1 codex.petra.ac.id > /dev/null 2>&1; then
    echo "ERROR: Cannot reach MQTT broker codex.petra.ac.id"
    echo "Please check your internet connection and try again."
    exit 1
fi

echo "✓ MQTT broker is reachable"

# Check if Python dependencies are available
echo "Checking Python dependencies..."
if ! python3 -c "import rplidar" 2>/dev/null; then
    echo "ERROR: rplidar-roboticia not installed"
    echo "Please install with: pip3 install rplidar-roboticia"
    exit 1
fi

if ! python3 -c "import paho.mqtt.client" 2>/dev/null; then
    echo "ERROR: paho-mqtt not installed"
    echo "Please install with: pip3 install paho-mqtt"
    exit 1
fi

echo "✓ Python dependencies are available"

# Check if test_robot.py exists
if [ ! -f "test_robot.py" ]; then
    echo "ERROR: test_robot.py not found in current directory"
    echo "Please make sure you're in the correct directory"
    exit 1
fi

echo "✓ test_robot.py found"

echo ""
echo "Starting test_robot.py..."
echo "This will send lidar data to MQTT broker for mapping"
echo "Press Ctrl+C to stop"

# Start test_robot.py
python3 test_robot.py 