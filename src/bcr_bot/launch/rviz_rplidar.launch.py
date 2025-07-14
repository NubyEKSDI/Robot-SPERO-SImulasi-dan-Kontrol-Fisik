#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch configurations
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Node untuk RPLIDAR
    rplidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_composition',
        name='rplidar_a1m8',
        output='screen',
        parameters=[
            {'serial_port': serial_port},
            {'frame_id': frame_id},
            {'inverted': False},
            {'angle_compensate': True},
            {'scan_mode': 'Standard'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Node untuk RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('bcr_bot'), 'rviz', 'rplidar.rviz')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port untuk RPLIDAR (misal: /dev/ttyUSB0)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame id untuk data LIDAR'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        rplidar_node,
        rviz_node
    ]) 