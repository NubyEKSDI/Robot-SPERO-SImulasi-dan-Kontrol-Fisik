#!/bin/bash

# MQTT Robustness Test for LiDAR A1M8
# This script tests the robust MQTT connection handling

echo "=========================================="
echo "    MQTT ROBUSTNESS TEST FOR LIDAR A1M8"
echo "=========================================="
echo ""
echo "This test will verify:"
echo "- MQTT connection stability"
echo "- Automatic reconnection with exponential backoff"
echo "- LiDAR data transmission during disconnections"
echo "- Proper error handling and recovery"
echo ""

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
    echo "Shutting down MQTT test..."
    if [ ! -z "$TEST_PID" ]; then
        kill $TEST_PID 2>/dev/null
        echo "✓ MQTT test stopped"
    fi
    echo "✓ MQTT test shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "Starting MQTT robustness test..."

# Run the MQTT robustness test
python3 src/bcr_bot/scripts/test_mqtt_robustness.py &
TEST_PID=$!

echo ""
echo "✓ MQTT robustness test started"
echo "  - Test PID: $TEST_PID"
echo ""
echo "The test will:"
echo "1. Connect to MQTT broker"
echo "2. Send test LiDAR data"
echo "3. Simulate disconnections"
echo "4. Test automatic reconnection"
echo "5. Verify data transmission recovery"
echo ""
echo "Press Ctrl+C to stop"

# Wait for background processes
wait 