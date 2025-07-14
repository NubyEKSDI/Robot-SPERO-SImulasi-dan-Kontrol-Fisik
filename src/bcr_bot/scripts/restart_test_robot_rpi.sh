#!/bin/bash

# Script untuk restart test_robot.py di Raspberry Pi
# Usage: ./restart_test_robot_rpi.sh
# 
# Script ini harus dijalankan di Raspberry Pi

echo "=== BCR Bot Test Robot Restart (Raspberry Pi) ==="

# Kill existing test_robot.py processes
echo "Stopping existing test_robot.py processes..."
pkill -f "test_robot.py" || true

# Wait a moment for clean shutdown
sleep 3

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

# Start test_robot.py
echo "Starting test_robot.py..."
cd /home/pi/bcr_ws  # Adjust path as needed
source install/setup.bash
python3 src/bcr_bot/scripts/test_robot.py &
TEST_ROBOT_PID=$!

echo "test_robot.py started with PID: $TEST_ROBOT_PID"
echo ""
echo "=== Restart Complete ==="
echo "Check the following:"
echo "1. Should show 'MQTT: Connected successfully'"
echo "2. Should show 'LiDAR initialized successfully'"
echo "3. Should show 'Waiting for roam.py ready signal'"
echo "4. When roam.py connects, should show 'LiDAR streaming started'"
echo ""
echo "If roam.py is already running, you should see:"
echo "- 'Detected roam.py reconnection!'"
echo "- 'Restarting lidar streaming for roam.py...'"
echo ""
echo "Press Ctrl+C to stop test_robot.py" 