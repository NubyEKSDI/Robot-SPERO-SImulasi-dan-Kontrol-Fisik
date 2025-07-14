#!/bin/bash

# Quick restart script untuk roam.py
# Usage: ./quick_restart_roam.sh

echo "=== Quick Restart roam.py ==="

# Kill existing roam.py processes
echo "🛑 Stopping existing roam.py processes..."
pkill -f "roam.py" || true

# Wait for clean shutdown
echo "⏳ Waiting for clean shutdown..."
sleep 3

# Check if processes are still running
if pgrep -f "roam.py" > /dev/null; then
    echo "⚠️  roam.py still running, force killing..."
    pkill -9 -f "roam.py" || true
    sleep 2
fi

# Start roam.py
echo "🚀 Starting roam.py..."
cd /home/nuby/bcr_ws
source install/setup.bash

# Start in background and capture PID
ros2 run bcr_bot roam &
ROAM_PID=$!

echo "✅ roam.py started with PID: $ROAM_PID"
echo ""
echo "📋 Check the following in roam.py output:"
echo "1. 'MQTT: Connected to broker'"
echo "2. 'MQTT: All topics subscribed successfully'"
echo "3. 'MQTT: Sent ready message to LiDAR systems (reconnect)'"
echo ""
echo "🔍 To monitor roam.py:"
echo "   tail -f /tmp/roam.log (if logging enabled)"
echo ""
echo "🛑 To stop roam.py:"
echo "   kill $ROAM_PID"
echo ""
echo "📡 If MQTT still disconnects, check:"
echo "   python3 src/bcr_bot/scripts/check_mqtt_status.py" 