#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bcr = get_package_share_directory('bcr_bot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set environment variables
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.path.join(pkg_bcr, 'worlds') + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = os.path.join(pkg_bcr, 'models') + ':' + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    
    # Launch Ignition with the scaled world
    ign_gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            os.path.join(pkg_bcr, 'worlds', 'gedung_p', 'gedung_p.world')
        ],
        output='screen'
    )
    
    # Spawn the robot with a delay to ensure the world is loaded
    spawn_bcr_bot_node = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bcr, 'launch', 'bcr_bot_ign_spawn.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'position_x': '0.0',
                    'position_y': '0.0',
                    'orientation_yaw': '0.0'
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        ign_gazebo,
        spawn_bcr_bot_node
    ]) 