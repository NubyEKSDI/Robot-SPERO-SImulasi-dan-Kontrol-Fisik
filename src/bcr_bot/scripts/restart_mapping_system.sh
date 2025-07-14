#!/bin/bash

# Script untuk restart sistem mapping ketika ada masalah koneksi
# Usage: ./restart_mapping_system.sh [roam|test_robot|both]
# 
# Options:
#   roam      - Restart hanya roam.py
#   test_robot - Restart hanya test_robot.py di Raspberry Pi
#   both      - Restart keduanya (default)

echo "=== BCR Bot Mapping System Restart ==="

# Default action
ACTION=${1:-both}

case $ACTION in
    "roam")
        echo "Restarting roam.py only..."
        
        # Kill existing roam.py processes
        echo "Stopping existing roam.py processes..."
        pkill -f "roam.py" || true
        
        # Wait a moment
        sleep 2
        
        # Start roam.py
        echo "Starting roam.py..."
        cd /home/nuby/bcr_ws
        source install/setup.bash
        ros2 run bcr_bot roam &
        ROAM_PID=$!
        echo "roam.py started with PID: $ROAM_PID"
        
        echo "✓ roam.py restarted successfully"
        ;;
        
    "test_robot")
        echo "Restarting test_robot.py on Raspberry Pi..."
        echo "Please run this command on the Raspberry Pi:"
        echo "cd /path/to/bcr_ws && python3 src/bcr_bot/scripts/test_robot.py"
        echo ""
        echo "Or use the start script:"
        echo "./src/bcr_bot/scripts/start_test_robot_rpi.sh"
        ;;
        
    "both")
        echo "Restarting both roam.py and test_robot.py..."
        
        # Kill existing roam.py processes
        echo "Stopping existing roam.py processes..."
        pkill -f "roam.py" || true
        
        # Wait a moment
        sleep 2
        
        # Start roam.py
        echo "Starting roam.py..."
        cd /home/nuby/bcr_ws
        source install/setup.bash
        ros2 run bcr_bot roam &
        ROAM_PID=$!
        echo "roam.py started with PID: $ROAM_PID"
        
        echo ""
        echo "✓ roam.py restarted successfully"
        echo ""
        echo "Now restart test_robot.py on Raspberry Pi:"
        echo "1. SSH to Raspberry Pi"
        echo "2. Run: cd /path/to/bcr_ws && python3 src/bcr_bot/scripts/test_robot.py"
        echo "3. Or use: ./src/bcr_bot/scripts/start_test_robot_rpi.sh"
        ;;
        
    *)
        echo "Invalid option: $ACTION"
        echo "Usage: $0 [roam|test_robot|both]"
        echo "  roam      - Restart hanya roam.py"
        echo "  test_robot - Restart hanya test_robot.py di Raspberry Pi"
        echo "  both      - Restart keduanya (default)"
        exit 1
        ;;
esac

echo ""
echo "=== Restart Complete ==="
echo "Check the following:"
echo "1. roam.py should show 'MQTT: Connected to broker'"
echo "2. test_robot.py should show 'MQTT: Connected successfully'"
echo "3. test_robot.py should show 'LiDAR streaming started'"
echo "4. roam.py should show '📡 Real-World LiDAR: Front=X.XXm'"
echo ""
echo "If issues persist, check:"
echo "- MQTT broker connectivity (codex.petra.ac.id)"
echo "- LiDAR device connection (/dev/ttyUSB0)"
echo "- Network connectivity between devices" 