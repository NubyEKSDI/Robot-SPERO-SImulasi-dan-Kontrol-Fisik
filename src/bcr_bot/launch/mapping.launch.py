import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_bcr = get_package_share_directory('bcr_bot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    use_physical_lidar = LaunchConfiguration('use_physical_lidar', default='False')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the slam_toolbox stack'
    )

    declare_use_physical_lidar = DeclareLaunchArgument(
        'use_physical_lidar',
        default_value='false',
        description='Use physical A1M8 lidar via MQTT from test_robot.py'
    )

    # MQTT Bridge node to receive lidar data from test_robot.py
    mqtt_bridge_node = Node(
        package='bcr_bot',
        executable='roam',
        name='mqtt_bridge_node',
        output='screen',
        condition=LaunchConfiguration('use_physical_lidar'),
        parameters=[{
            'use_sim_time': use_sim_time,
            'mqtt_broker': 'codex.petra.ac.id',
            'mqtt_port': 1883,
        }]
    )

    # Include slam_toolbox launch file with different configs based on lidar type
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_bcr, 'config', 'mapper_params_online_async.yaml'),
        }.items()
    )

    # Include slam_toolbox launch file for physical lidar
    slam_toolbox_physical_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_bcr, 'config', 'mapper_params_physical_lidar.yaml'),
        }.items(),
        condition=LaunchConfiguration('use_physical_lidar')
    )

    # Launch RViz
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(pkg_bcr, 'rviz', 'map.rviz')
        ]
    )

    # Static transform publisher for physical lidar
    physical_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_physical_lidar',
        output='screen',
        arguments=['0', '0', '0.25', '0', '0', '0', 'base_link', 'physical_robot_laser'],
        condition=LaunchConfiguration('use_physical_lidar')
    )

    # Static transform publisher for simulation
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_physical_lidar)
    ld.add_action(mqtt_bridge_node)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(slam_toolbox_physical_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(physical_lidar_tf_node)
    ld.add_action(static_transform_publisher_node)

    return ld
