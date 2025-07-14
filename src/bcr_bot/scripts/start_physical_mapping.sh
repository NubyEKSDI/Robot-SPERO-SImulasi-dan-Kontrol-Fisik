#!/bin/bash

# Script untuk memulai mapping dengan lidar A1M8 fisik via MQTT
# Usage: ./start_physical_mapping.sh
# 
# Pastikan test_robot.py sudah berjalan di Raspberry Pi dan mengirim data lidar via MQTT

echo "=== Starting BCR Bot Mapping with Physical A1M8 Lidar via MQTT ==="
echo "Prerequisites:"
echo "1. test_robot.py sudah berjalan di Raspberry Pi"
echo "2. A1M8 lidar terhubung ke Raspberry Pi di /dev/ttyUSB0"
echo "3. MQTT broker (codex.petra.ac.id) dapat diakses"
echo "4. Robot fisik siap dan aman"
echo "5. Area mapping sudah disiapkan"
echo ""

# Check MQTT connectivity
echo "Checking MQTT connectivity..."
if ! ping -c 1 codex.petra.ac.id > /dev/null 2>&1; then
    echo "ERROR: Cannot reach MQTT broker codex.petra.ac.id"
    echo "Please check your internet connection and try again."
    exit 1
fi

echo "✓ MQTT broker is reachable"

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 is not installed or not in PATH"
    echo "Please install ROS2 and source the workspace"
    exit 1
fi

echo "✓ ROS2 is available"

echo ""
echo "Starting mapping with physical lidar via MQTT..."
echo "Make sure test_robot.py is running on Raspberry Pi"
echo "Press Ctrl+C to stop"

# Launch mapping with physical lidar via MQTT
ros2 launch bcr_bot mapping.launch.py use_physical_lidar:=true use_sim_time:=false 