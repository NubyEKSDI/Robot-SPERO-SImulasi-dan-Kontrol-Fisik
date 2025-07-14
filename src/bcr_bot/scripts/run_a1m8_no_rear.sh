#!/bin/bash

# A1M8 LiDAR System - NO REAR DETECTION
# This script runs the A1M8 LiDAR system with rear sector exclusion

echo "=========================================="
echo "    A1M8 LIDAR SYSTEM - NO REAR DETECTION"
echo "=========================================="
echo ""
echo "Configuration:"
echo "- Rear sector excluded: 135° to 225°"
echo "- Front detection: 0° ± 10°"
echo "- Left detection: 90° ± 10°"
echo "- Right detection: 270° ± 10°"
echo "- MQTT topic: robot/lidar/real_world_data"
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 is not installed or not in PATH"
    exit 1
fi

# Check if we're in a ROS2 workspace
if [ ! -f "install/setup.bash" ]; then
    echo "ERROR: Please run this script from the ROS2 workspace root"
    echo "Current directory: $(pwd)"
    exit 1
fi

# Source the workspace
echo "Sourcing ROS2 workspace..."
source install/setup.bash

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Shutting down A1M8 LiDAR system..."
    if [ ! -z "$LIDAR_PID" ]; then
        kill $LIDAR_PID 2>/dev/null
        echo "✓ LiDAR processor stopped"
    fi
    if [ ! -z "$SENDER_PID" ]; then
        kill $SENDER_PID 2>/dev/null
        echo "✓ Data sender stopped"
    fi
    if [ ! -z "$TEST_PID" ]; then
        kill $TEST_PID 2>/dev/null
        echo "✓ Test sender stopped"
    fi
    echo "✓ A1M8 LiDAR system shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "Starting A1M8 LiDAR system..."

# Option 1: Run with real A1M8 LiDAR data (if available)
if [ "$1" = "real" ]; then
    echo "Mode: Real A1M8 LiDAR data"
    echo "Starting LiDAR processor..."
    python3 src/bcr_bot/scripts/lidar_a1m8_no_rear.py &
    LIDAR_PID=$!
    
    echo "Starting data sender..."
    python3 src/bcr_bot/scripts/send_a1m8_no_rear.py &
    SENDER_PID=$!
    
    echo ""
    echo "✓ A1M8 LiDAR system started with real data"
    echo "  - LiDAR processor PID: $LIDAR_PID"
    echo "  - Data sender PID: $SENDER_PID"
    echo ""
    echo "Press Ctrl+C to stop"

# Option 2: Run with test data (default)
else
    echo "Mode: Test A1M8 LiDAR data"
    echo "Starting test data sender..."
    python3 src/bcr_bot/scripts/test_a1m8_no_rear.py &
    TEST_PID=$!
    
    echo ""
    echo "✓ A1M8 LiDAR test system started"
    echo "  - Test sender PID: $TEST_PID"
    echo ""
    echo "Available test scenarios:"
    echo "  clear: Clear path"
    echo "  front_obstacle: Front obstacle"
    echo "  left_obstacle: Left obstacle"
    echo "  right_obstacle: Right obstacle"
    echo "  both_sides: Both sides blocked"
    echo "  close_front: Close front obstacle"
    echo ""
    echo "Commands:"
    echo "  [scenario_name]: Switch scenario"
    echo "  s: Send current scenario"
    echo "  c: Toggle continuous sending"
    echo "  q: Quit"
    echo ""
    echo "Press Ctrl+C to stop"
fi

# Wait for background processes
wait 