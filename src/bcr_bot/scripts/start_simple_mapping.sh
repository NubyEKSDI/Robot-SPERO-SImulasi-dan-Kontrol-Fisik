#!/bin/bash

echo "============================================="
echo "🗺️  BCR BOT SIMPLE GLOBAL MAPPING"
echo "============================================="
echo ""
echo "FITUR SIMPLE MAPPING:"
echo "✅ Obstacle tetap pada posisi absolut"
echo "✅ Robot bisa berputar tanpa mempengaruhi map"
echo "✅ Data LiDAR dari robot fisik + Gazebo"
echo "✅ Visualisasi real-time di RViz"
echo ""
echo "============================================="

# Check if we're in the correct workspace
if [ ! -d "src/bcr_bot" ]; then
    echo "❌ Error: Please run this script from the ROS2 workspace root"
    echo "   Current directory: $(pwd)"
    echo "   Expected: ~/bcr_ws"
    exit 1
fi

# Source ROS2 environment
echo "🔧 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "🚀 STARTING SIMPLE MAPPING SYSTEM..."
echo ""
echo "KOMPONEN YANG AKAN DIJALANKAN:"
echo "1. MQTT to ROS2 Bridge - Terima data LiDAR real-world"
echo "2. Transform Publisher - Koordinat global"
echo "3. RViz2 - Visualisasi mapping"
echo ""
echo "🎯 TOPICS:"
echo "   /scan - LiDAR scan data (dari A1M8 real-world)"
echo "   /tf   - Transform data"
echo ""

read -p "Lanjutkan? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Dibatalkan oleh user"
    exit 1
fi

echo ""
echo "⏳ Building workspace..."
colcon build --packages-select bcr_bot
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
else
    echo "❌ Build failed! Please check for errors."
    exit 1
fi

# Source again after build
source install/setup.bash

echo ""
echo "🚀 Launching Simple Mapping..."
echo ""

# Start components in background
echo "📡 Starting MQTT to ROS2 Bridge..."
python3 src/bcr_bot/scripts/mqtt_to_ros2_bridge.py &
MQTT_PID=$!

sleep 2

echo "🗺️  Starting Transform Publisher..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TF_PID=$!

sleep 1

echo "🖥️  Starting RViz2..."
rviz2 -d src/bcr_bot/rviz/a1m8_mapping.rviz &
RVIZ_PID=$!

echo ""
echo "🎯 TIPS PENGGUNAAN:"
echo "   • Jalankan roam.py di terminal lain untuk menggerakkan robot"
echo "   • Di RViz, perhatikan data LiDAR dari robot fisik"
echo "   • Obstacle dari real-world akan terlihat di RViz"
echo "   • Map akan terupdate real-time"
echo ""
echo "⚠️  CATATAN PENTING:"
echo "   • Pastikan robot fisik sudah terhubung MQTT"
echo "   • Data LiDAR A1M8 akan ditampilkan di /scan topic"
echo "   • Tekan Ctrl+C untuk menghentikan semua komponen"
echo ""
echo "🚀 System running... Press Ctrl+C to stop"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Stopping all components..."
    
    if [ ! -z "$MQTT_PID" ]; then
        kill $MQTT_PID 2>/dev/null
        echo "✓ MQTT Bridge stopped"
    fi
    
    if [ ! -z "$TF_PID" ]; then
        kill $TF_PID 2>/dev/null
        echo "✓ Transform Publisher stopped"
    fi
    
    if [ ! -z "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null
        echo "✓ RViz stopped"
    fi
    
    echo "✅ All components stopped"
    echo "🏁 Simple Mapping System stopped"
    echo "============================================="
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT

# Wait for user interrupt
while true; do
    sleep 1
done 