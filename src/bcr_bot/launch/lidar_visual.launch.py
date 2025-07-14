#!/usr/bin/env python3
"""
Launch file untuk visualisasi RPLIDAR A1M8 dari robot fisik via MQTT
Menjalankan:
1. MQTT to ROS2 Bridge (subscribe MQTT, publish /scan)
2. RViz untuk visualisasi data lidar
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('bcr_bot')
    
    # RViz config file (opsional, bisa kosong jika tidak ada)
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'lidar_visual.rviz')
    
    # Launch arguments
    use_rviz_config = DeclareLaunchArgument(
        'use_rviz_config',
        default_value='false',
        description='Whether to use RViz config file'
    )
    
    return LaunchDescription([
        use_rviz_config,
        
        # MQTT to ROS2 Bridge Node
        Node(
            package='bcr_bot',
            executable='mqtt_to_ros2_bridge',
            name='mqtt_to_ros2_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Static Transform Publisher (laser frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        ),
    ]) 