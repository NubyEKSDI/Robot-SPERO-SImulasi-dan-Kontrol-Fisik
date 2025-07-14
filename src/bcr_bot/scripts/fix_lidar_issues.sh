#!/bin/bash
# Script untuk mengatasi masalah LiDAR RP-Lidar A1M8

echo "=== FIXING RP-LIDAR A1M8 ISSUES ==="

# Function to check if LiDAR is connected
check_lidar_connection() {
    echo "Checking LiDAR connection..."
    if ls /dev/ttyUSB* 2>/dev/null; then
        echo "✓ USB devices found:"
        ls -la /dev/ttyUSB*
    else
        echo "✗ No USB devices found"
        return 1
    fi
    
    if lsusb | grep -i "10c4.*ea60" > /dev/null; then
        echo "✓ RP-Lidar A1M8 detected in USB"
        return 0
    else
        echo "✗ RP-Lidar A1M8 not detected in USB"
        return 1
    fi
}

# Function to fix permissions
fix_permissions() {
    echo "Fixing USB permissions..."
    sudo usermod -a -G dialout $USER
    
    for device in /dev/ttyUSB*; do
        if [ -e "$device" ]; then
            sudo chmod 666 "$device"
            echo "✓ Set permissions for $device"
        fi
    done
}

# Function to reset USB subsystem
reset_usb() {
    echo "Resetting USB subsystem..."
    sudo modprobe -r usbserial 2>/dev/null
    sudo modprobe usbserial
    echo "✓ USB subsystem reset"
}

# Function to kill existing LiDAR processes
kill_lidar_processes() {
    echo "Killing existing LiDAR processes..."
    pkill -f "test_robot.py" 2>/dev/null
    pkill -f "test_lidar.py" 2>/dev/null
    pkill -f "rplidar" 2>/dev/null
    echo "✓ Killed existing LiDAR processes"
}

# Function to test LiDAR with Python
test_lidar_python() {
    echo "Testing LiDAR with Python..."
    python3 -c "
import time
from rplidar import RPLidar

try:
    print('Connecting to LiDAR...')
    lidar = RPLidar('/dev/ttyUSB0', baudrate=115200, timeout=1.0)
    
    print('Getting LiDAR info...')
    info = lidar.get_info()
    print(f'LiDAR Info: {info}')
    
    print('Getting health status...')
    health = lidar.get_health()
    print(f'Health: {health}')
    
    print('Starting scan test...')
    lidar.start()
    time.sleep(1.0)
    
    scan_count = 0
    for scan in lidar.iter_scans():
        scan_count += 1
        print(f'Scan {scan_count}: {len(scan)} points')
        if scan_count >= 3:
            break
    
    lidar.stop()
    lidar.disconnect()
    print('✓ LiDAR test successful!')
    
except Exception as e:
    print(f'✗ LiDAR test failed: {e}')
    exit(1)
"
}

# Main execution
echo "Step 1: Checking LiDAR connection..."
if ! check_lidar_connection; then
    echo "LiDAR not detected. Please check:"
    echo "1. USB cable connection"
    echo "2. Power supply"
    echo "3. LED indicator on LiDAR"
    exit 1
fi

echo ""
echo "Step 2: Fixing permissions..."
fix_permissions

echo ""
echo "Step 3: Killing existing processes..."
kill_lidar_processes

echo ""
echo "Step 4: Resetting USB subsystem..."
reset_usb

echo ""
echo "Step 5: Waiting for device to stabilize..."
sleep 3

echo ""
echo "Step 6: Testing LiDAR..."
if test_lidar_python; then
    echo ""
    echo "=== LIDAR ISSUES FIXED ==="
    echo "✓ LiDAR is working properly"
    echo ""
    echo "You can now run:"
    echo "  python3 test_robot.py"
    echo ""
else
    echo ""
    echo "=== LIDAR STILL HAS ISSUES ==="
    echo "Try these additional steps:"
    echo "1. Reboot the system: sudo reboot"
    echo "2. Try different USB port"
    echo "3. Check LiDAR firmware"
    echo "4. Replace USB cable"
    exit 1
fi 