#!/usr/bin/env python3
"""
Launch file for A1M8 LiDAR SLAM mapping
This launch file is designed to work with real A1M8 LiDAR data received via MQTT
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('bcr_bot')
    
    # Path to config files
    mapper_params_file = os.path.join(package_dir, 'config', 'a1m8_mapper_params.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'a1m8_mapping.rviz')
    
    return LaunchDescription([
        # Note: test_robot.py should be running on Raspberry Pi, not launched from here
        
        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': False}]
        ),
        
        # Static transform: map -> odom (for mapping without odometry)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': False}]
        ),
        
        # Static transform: odom -> base_link (for mapping without odometry)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': False}]
        ),
        
        # SLAM Toolbox for mapping
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                mapper_params_file,
                {'use_sim_time': False}
            ]
        ),
        
        # Delayed RViz launch (wait for SLAM toolbox to start)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': False}]
                )
            ]
        ),
        
        # Print instructions
        ExecuteProcess(
            cmd=['echo', 'A1M8 SLAM Mapping started! Make sure:'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', '1. Raspberry Pi is running test_robot.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', '2. MQTT to ROS2 bridge is running'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['echo', '3. Robot is publishing to /scan topic'],
            output='screen'
        ),
    ]) 