#!/bin/bash

# Optimize ROS2 for Virtual Machine environment
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Suppress transport errors and reduce log noise
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=20  # Only warnings and errors
export RCUTILS_COLORIZED_OUTPUT=1

# Reduce FastRTPS verbosity  
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null

# Check if mapping mode is requested
MAPPING_MODE="false"
if [[ "$1" == "--mapping" ]] || [[ "$1" == "-m" ]]; then
    MAPPING_MODE="true"
    echo "=== Starting RPLIDAR A1M8 SLAM Mapping System ==="
    echo "1. MQTT to ROS2 Bridge"
    echo "2. Scan to PointCloud Converter (for persistence)"
    echo "3. SLAM Toolbox for mapping"
    echo "4. RViz for mapping visualization with robot tracking"
    echo "=================================================="
else
    echo "=== Starting RPLIDAR A1M8 Visualization System ==="
    echo "1. MQTT to ROS2 Bridge"
    echo "2. Scan to PointCloud Converter (for persistence)"
    echo "3. RViz for visualization with robot orientation"
    echo "Usage: $0 [--mapping | -m] for SLAM mapping mode"
    echo "=================================================="
fi

# Source ROS2 environment
source ~/bcr_ws/install/setup.bash

# Function to cleanup on exit
cleanup() {
    echo "Stopping all processes..."
    kill $BRIDGE_PID $RVIZ_PID $TF_PID $SLAM_PID $POINTCLOUD_PID 2>/dev/null
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM

# Start MQTT to ROS2 Bridge in background
echo "Starting MQTT to ROS2 Bridge..."
if [[ "$MAPPING_MODE" == "true" ]]; then
    echo "Bridge starting in MAPPING mode (reduced frequency for SLAM)"
    MAPPING_MODE=true python3 ~/bcr_ws/src/bcr_bot/scripts/mqtt_to_ros2_bridge.py &
else
    echo "Bridge starting in VISUALIZATION mode (normal frequency)"
    python3 ~/bcr_ws/src/bcr_bot/scripts/mqtt_to_ros2_bridge.py &
fi
BRIDGE_PID=$!

# Wait a moment for bridge to start
sleep 3

# Start static transform publisher for laser frame (new argument format)
echo "Starting static transform publisher..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_link --child-frame-id laser &
TF_PID=$!

# Start scan to pointcloud converter for persistence and robot visualization
echo "Starting scan to pointcloud converter..."
python3 ~/bcr_ws/src/bcr_bot/scripts/scan_to_pointcloud.py &
POINTCLOUD_PID=$!

# If mapping mode is enabled, start additional transforms and SLAM
if [[ "$MAPPING_MODE" == "true" ]]; then
    # Additional transforms for mapping (new argument format)
    echo "Starting additional transforms for mapping..."
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 \
        --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id map --child-frame-id odom &
    TF_MAP_PID=$!
    
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 \
        --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id odom --child-frame-id base_link &
    TF_ODOM_PID=$!
    
    # Start SLAM Toolbox with improved queue settings
    echo "Starting SLAM Toolbox with optimized settings..."
    ros2 run slam_toolbox sync_slam_toolbox_node \
        --ros-args \
        --params-file ~/bcr_ws/src/bcr_bot/config/a1m8_mapper_params.yaml \
        -p use_sim_time:=false \
        --remap scan:=/scan \
        --log-level warn &
    SLAM_PID=$!
    
    # Wait for SLAM to start
    sleep 3
    
    # Start RViz with mapping configuration
    echo "Starting RViz with mapping configuration..."
    rviz2 -d ~/bcr_ws/src/bcr_bot/rviz/a1m8_mapping.rviz &
    RVIZ_PID=$!
    
    echo "=================================================="
    echo "A1M8 SLAM Mapping System started successfully!"
    echo "- MQTT Bridge PID: $BRIDGE_PID"
    echo "- SLAM Toolbox PID: $SLAM_PID"
    echo "- RViz PID: $RVIZ_PID"
    echo "- PointCloud Converter PID: $POINTCLOUD_PID"
    echo "- Transform Publishers: $TF_PID, $TF_MAP_PID, $TF_ODOM_PID"
    echo ""
    echo "Features:"
    echo "- Real-time A1M8 LiDAR SLAM mapping"
    echo "- Persistent scan points for better visualization"
    echo "- Robot orientation and path tracking"
    echo "- Map building and loop closure"
    echo ""
    echo "Make sure:"
    echo "1. Raspberry Pi is running test_robot.py"
    echo "2. Robot is sending data to MQTT topic: robot/lidar/real_world_data"
    echo "3. Move the robot around to build the map"
    echo ""
    echo "To save the map:"
    echo "  ros2 run nav2_map_server map_saver_cli -f my_map"
    echo ""
    echo "Press Ctrl+C to stop all processes"
    echo "=================================================="
    
    # Wait for processes
    wait $BRIDGE_PID $RVIZ_PID $TF_PID $SLAM_PID $TF_MAP_PID $TF_ODOM_PID $POINTCLOUD_PID
else
    # Start RViz with visualization only
    echo "Starting RViz..."
    rviz2 -d ~/bcr_ws/src/bcr_bot/rviz/lidar_visual.rviz &
    RVIZ_PID=$!
    
    echo "=================================================="
    echo "A1M8 Visualization System started successfully!"
    echo "- MQTT Bridge PID: $BRIDGE_PID"
    echo "- RViz PID: $RVIZ_PID"
    echo "- PointCloud Converter PID: $POINTCLOUD_PID"
    echo "- Transform Publisher PID: $TF_PID"
    echo ""
    echo "Features:"
    echo "- Real-time A1M8 LiDAR visualization (red dots)"
    echo "- Persistent scan points (white dots with fade)"
    echo "- Robot orientation indicator (yellow footprint with arrow)"
    echo "- Robot path tracking (green line)"
    echo ""
    echo "Make sure robot is sending data to MQTT topic:"
    echo "  robot/lidar/real_world_data"
    echo ""
    echo "Press Ctrl+C to stop all processes"
    echo "=================================================="
    
    # Wait for processes
    wait $BRIDGE_PID $RVIZ_PID $TF_PID $POINTCLOUD_PID
fi 