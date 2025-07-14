#!/bin/bash

# BCR Bot Global Mapping Startup Script - FIXED VERSION
# This script starts all components needed for global mapping:
# 1. MQTT-ROS2 Bridge (for A1M8 LiDAR data)
# 2. Global Mapping Node (with fixed transforms)
# 3. RViz with global mapping configuration
# 4. Diagnostic tools

echo "=== BCR BOT GLOBAL MAPPING STARTUP - FIXED VERSION ==="

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "=== SHUTTING DOWN GLOBAL MAPPING ==="
    echo "Stopping all processes..."
    
    # Kill all background processes
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Stopping process $pid..."
            kill "$pid"
        fi
    done
    
    # Wait a moment for graceful shutdown
    sleep 2
    
    # Force kill if needed
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Force stopping process $pid..."
            kill -9 "$pid"
        fi
    done
    
    echo "✓ All processes stopped"
    exit 0
}

# Setup signal handlers
trap cleanup SIGINT SIGTERM

# Array to track background process PIDs
pids=()

# Setup environment
source /opt/ros/humble/setup.bash
cd /home/nuby/bcr_ws
source install/setup.bash

echo ""
echo "=== STEP 1: STARTING MQTT-ROS2 BRIDGE ==="
echo "Starting MQTT-ROS2 bridge for A1M8 LiDAR data..."
python3 src/bcr_bot/scripts/mqtt_to_ros2_bridge.py &
mqtt_bridge_pid=$!
pids+=($mqtt_bridge_pid)
echo "✓ MQTT-ROS2 Bridge started (PID: $mqtt_bridge_pid)"

# Wait for bridge to initialize
sleep 3

echo ""
echo "=== STEP 2: STARTING GLOBAL MAPPING NODE ==="
echo "Starting global mapping node with fixed transforms..."
python3 src/bcr_bot/scripts/global_mapping_node.py &
global_mapping_pid=$!
pids+=($global_mapping_pid)
echo "✓ Global Mapping Node started (PID: $global_mapping_pid)"

# Wait for global mapping to initialize
sleep 5

echo ""
echo "=== STEP 3: STARTING DIAGNOSTIC TOOL ==="
echo "Starting global mapping diagnostic tool..."
python3 src/bcr_bot/scripts/diagnose_global_mapping.py &
diagnostic_pid=$!
pids+=($diagnostic_pid)
echo "✓ Diagnostic Tool started (PID: $diagnostic_pid)"

# Wait for diagnostics to run
sleep 3

echo ""
echo "=== STEP 4: STARTING RVIZ ==="
echo "Starting RViz with global mapping configuration..."

# Check if RViz config exists
RVIZ_CONFIG="src/bcr_bot/config/global_mapping.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
    echo "Using RViz config: $RVIZ_CONFIG"
    rviz2 -d "$RVIZ_CONFIG" &
    rviz_pid=$!
    pids+=($rviz_pid)
    echo "✓ RViz started (PID: $rviz_pid)"
else
    echo "⚠️ RViz config not found, starting with default config"
    rviz2 &
    rviz_pid=$!
    pids+=($rviz_pid)
    echo "✓ RViz started with default config (PID: $rviz_pid)"
fi

# Wait for RViz to load
sleep 5

echo ""
echo "=== SYSTEM STATUS ==="
echo "✓ MQTT-ROS2 Bridge: Running (PID: $mqtt_bridge_pid)"
echo "✓ Global Mapping Node: Running (PID: $global_mapping_pid)"
echo "✓ Diagnostic Tool: Running (PID: $diagnostic_pid)"
echo "✓ RViz: Running (PID: $rviz_pid)"

echo ""
echo "=== GLOBAL MAPPING SYSTEM READY ==="
echo ""
echo "WHAT TO EXPECT IN RVIZ:"
echo "  ✓ Robot should appear and move in RViz"
echo "  ✓ LiDAR data should update in real-time"
echo "  ✓ Obstacles should stay in absolute positions"
echo "  ✓ Global map should build as robot moves"
echo "  ✓ Robot trajectory should be visible"
echo ""
echo "TROUBLESHOOTING:"
echo "  • If robot doesn't move: Check odometry data"
echo "  • If no LiDAR data: Check A1M8 connection"
echo "  • If obstacles follow robot: Check transform chain"
echo "  • If transforms missing: Check diagnostic output"
echo ""
echo "DIAGNOSTIC OUTPUT: Check terminal for diagnostic messages every 5 seconds"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=" * 60

# Keep script running and monitor processes
while true; do
    sleep 5
    
    # Check if any critical process died
    critical_processes=($mqtt_bridge_pid $global_mapping_pid)
    
    for pid in "${critical_processes[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "⚠️ Critical process $pid died! Restarting system..."
            cleanup
            exit 1
        fi
    done
    
    # Show brief status
    echo "[$(date '+%H:%M:%S')] Global Mapping System: ✓ All processes running"
done 