#!/bin/bash

# A1M8 LiDAR Mapping Script
# This script starts mapping using A1M8 LiDAR sensor

echo "Starting A1M8 LiDAR Mapping..."
echo "Make sure your A1M8 LiDAR is connected and test_robot.py is configured correctly"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
cd /home/nuby/bcr_ws
source install/setup.bash

echo "Building bcr_bot package..."
colcon build --packages-select bcr_bot

echo "Starting A1M8 mapping..."
ros2 launch bcr_bot a1m8_mapping.launch.py

echo "Mapping completed. Check the generated map files." 