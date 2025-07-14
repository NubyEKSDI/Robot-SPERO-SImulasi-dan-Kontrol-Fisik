#!/bin/bash

# Script untuk mengecek status sistem mapping
# Usage: ./check_mapping_status.sh

echo "=== BCR Bot Mapping System Status Check ==="
echo ""

# Check MQTT broker connectivity
echo "1. Checking MQTT broker connectivity..."
if ping -c 1 codex.petra.ac.id > /dev/null 2>&1; then
    echo "   ✅ MQTT broker (codex.petra.ac.id) is reachable"
else
    echo "   ❌ MQTT broker (codex.petra.ac.id) is not reachable"
    echo "   Please check network connectivity"
fi

# Check if roam.py is running
echo ""
echo "2. Checking roam.py status..."
if pgrep -f "roam.py" > /dev/null; then
    ROAM_PID=$(pgrep -f "roam.py")
    echo "   ✅ roam.py is running (PID: $ROAM_PID)"
else
    echo "   ❌ roam.py is not running"
    echo "   Start with: ros2 run bcr_bot roam"
fi

# Check if test_robot.py is running (on this machine)
echo ""
echo "3. Checking test_robot.py status (local)..."
if pgrep -f "test_robot.py" > /dev/null; then
    TEST_ROBOT_PID=$(pgrep -f "test_robot.py")
    echo "   ✅ test_robot.py is running locally (PID: $TEST_ROBOT_PID)"
else
    echo "   ℹ️  test_robot.py is not running locally (should be on Raspberry Pi)"
fi

# Check lidar device (if on Raspberry Pi)
echo ""
echo "4. Checking lidar device..."
if [ -e "/dev/ttyUSB0" ]; then
    if [ -r "/dev/ttyUSB0" ]; then
        echo "   ✅ Lidar device /dev/ttyUSB0 exists and is readable"
    else
        echo "   ⚠️  Lidar device /dev/ttyUSB0 exists but not readable"
        echo "   Try: sudo chmod 666 /dev/ttyUSB0"
    fi
else
    echo "   ❌ Lidar device /dev/ttyUSB0 not found"
    echo "   Please check lidar connection"
fi

# Check ROS2 topics
echo ""
echo "5. Checking ROS2 topics..."
if command -v ros2 > /dev/null 2>&1; then
    echo "   Checking /bcr_bot/scan topic..."
    if timeout 5 ros2 topic list | grep -q "/bcr_bot/scan"; then
        echo "   ✅ /bcr_bot/scan topic exists"
        
        # Check if topic is publishing
        echo "   Checking if topic is publishing data..."
        if timeout 3 ros2 topic echo /bcr_bot/scan --once > /dev/null 2>&1; then
            echo "   ✅ /bcr_bot/scan topic is publishing data"
        else
            echo "   ⚠️  /bcr_bot/scan topic exists but no data received"
        fi
    else
        echo "   ❌ /bcr_bot/scan topic not found"
    fi
else
    echo "   ❌ ROS2 not found or not sourced"
fi

# Check MQTT topics (if mosquitto_pub/sub is available)
echo ""
echo "6. Checking MQTT topics..."
if command -v mosquitto_pub > /dev/null 2>&1; then
    echo "   Testing MQTT publish..."
    if timeout 5 mosquitto_pub -h codex.petra.ac.id -t "test/status_check" -m "test" > /dev/null 2>&1; then
        echo "   ✅ MQTT publish test successful"
    else
        echo "   ❌ MQTT publish test failed"
    fi
else
    echo "   ℹ️  mosquitto-clients not installed, skipping MQTT test"
fi

echo ""
echo "=== Status Summary ==="

# Count issues
ISSUES=0
if ! ping -c 1 codex.petra.ac.id > /dev/null 2>&1; then
    ISSUES=$((ISSUES + 1))
fi
if ! pgrep -f "roam.py" > /dev/null; then
    ISSUES=$((ISSUES + 1))
fi
if [ ! -e "/dev/ttyUSB0" ]; then
    ISSUES=$((ISSUES + 1))
fi

if [ $ISSUES -eq 0 ]; then
    echo "✅ All systems appear to be working correctly"
    echo ""
    echo "Next steps:"
    echo "1. Start roam.py: ros2 run bcr_bot roam"
    echo "2. Start test_robot.py on Raspberry Pi"
    echo "3. Check for lidar data in roam.py output"
else
    echo "⚠️  Found $ISSUES potential issue(s)"
    echo ""
    echo "Troubleshooting steps:"
    echo "1. Check network connectivity to codex.petra.ac.id"
    echo "2. Ensure lidar is connected to /dev/ttyUSB0"
    echo "3. Start roam.py and test_robot.py"
    echo "4. Use restart scripts if needed:"
    echo "   ./src/bcr_bot/scripts/restart_mapping_system.sh"
fi

echo ""
echo "For detailed troubleshooting, see:"
echo "src/bcr_bot/scripts/TROUBLESHOOTING_MAPPING.md" 