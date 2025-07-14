#!/bin/bash

# A1M8 Robust LiDAR Handler
# This script runs the A1M8 LiDAR with robust MQTT connection handling

echo "=========================================="
echo "    A1M8 ROBUST LIDAR HANDLER"
echo "=========================================="
echo ""
echo "Features:"
echo "- Never disconnects from MQTT"
echo "- Automatic reconnection with exponential backoff"
echo "- Health monitoring and status reporting"
echo "- Robust error handling"
echo "- Excludes rear sector (135°-225°)"
echo ""

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Shutting down A1M8 LiDAR..."
    if [ ! -z "$LIDAR_PID" ]; then
        kill $LIDAR_PID 2>/dev/null
        echo "✓ A1M8 LiDAR stopped"
    fi
    echo "✓ A1M8 LiDAR shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "Starting A1M8 robust LiDAR handler..."

# Run the A1M8 robust LiDAR handler
python3 src/bcr_bot/scripts/a1m8_robust_lidar.py &
LIDAR_PID=$!

echo ""
echo "✓ A1M8 robust LiDAR handler started"
echo "  - Process PID: $LIDAR_PID"
echo ""
echo "The handler will:"
echo "1. Connect to MQTT broker"
echo "2. Send LiDAR data continuously"
echo "3. Auto-reconnect if disconnected"
echo "4. Monitor health and status"
echo "5. Never give up trying to reconnect"
echo ""
echo "Press Ctrl+C to stop"

# Wait for background processes
wait 