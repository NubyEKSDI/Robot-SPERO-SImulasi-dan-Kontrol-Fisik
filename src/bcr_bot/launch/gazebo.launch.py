#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    camera_enabled = LaunchConfiguration('camera_enabled', default='false')
    stereo_camera_enabled = LaunchConfiguration('stereo_camera_enabled', default='false')
    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled', default='true')
    odometry_source = LaunchConfiguration('odometry_source', default='world')
    
    world_file = LaunchConfiguration("world_file", default=join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                ' camera_enabled:=', camera_enabled,
                ' stereo_camera_enabled:=', stereo_camera_enabled,
                ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                ' odometry_source:=', odometry_source,
                ' sim_gazebo:=true'
            ])
        }]
    )
    
    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui', default='true'),
            'verbose': LaunchConfiguration('verbose', default='false')
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'bcr_bot',
                   '-z', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('camera_enabled', 
                              default_value='false', 
                              description='Enable camera sensor'),
        DeclareLaunchArgument('stereo_camera_enabled', 
                              default_value='false', 
                              description='Enable stereo camera sensor'),
        DeclareLaunchArgument('two_d_lidar_enabled', 
                              default_value='true', 
                              description='Enable 2D LiDAR sensor'),
        DeclareLaunchArgument('odometry_source', 
                              default_value='world', 
                              description='Odometry source: world or encoder'),
        DeclareLaunchArgument('world', 
                              default_value=world_file,
                              description='World file to load'),
        DeclareLaunchArgument('gui', 
                              default_value='true',
                              description='Launch Gazebo GUI'),
        DeclareLaunchArgument('verbose', 
                              default_value='false',
                              description='Verbose output'),
        DeclareLaunchArgument('use_sim_time', 
                              default_value='true',
                              description='Use simulation time'),
        
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(bcr_bot_path, "models")),

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(bcr_bot_path, "worlds")),
        
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
