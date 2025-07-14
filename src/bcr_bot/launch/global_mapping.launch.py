#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) time'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('bcr_bot'),
            'rviz',
            'global_mapping.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Global Mapping Node
    global_mapping_node = Node(
        package='bcr_bot',
        executable='global_mapping_node',
        name='global_mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/bcr_bot/odom', '/bcr_bot/odom'),
        ]
    )
    
    # MQTT to ROS2 Bridge (for real LiDAR data)
    mqtt_bridge_node = Node(
        package='bcr_bot',
        executable='mqtt_to_ros2_bridge',
        name='mqtt_to_ros2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'mapping_mode': True  # Enable mapping mode via parameter
        }]
    )
    
    # Static transform publisher (map -> odom)
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': '''<?xml version="1.0"?>
                <robot name="bcr_bot">
                    <link name="base_link">
                        <visual>
                            <geometry>
                                <box size="0.3 0.3 0.1"/>
                            </geometry>
                            <material name="blue">
                                <color rgba="0 0 1 1"/>
                            </material>
                        </visual>
                    </link>
                    <link name="laser">
                        <visual>
                            <geometry>
                                <cylinder radius="0.05" length="0.02"/>
                            </geometry>
                            <material name="red">
                                <color rgba="1 0 0 1"/>
                            </material>
                        </visual>
                    </link>
                    <joint name="laser_joint" type="fixed">
                        <parent link="base_link"/>
                        <child link="laser"/>
                        <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
                    </joint>
                </robot>'''
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        global_mapping_node,
        mqtt_bridge_node,
        static_transform_map_odom,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ]) 