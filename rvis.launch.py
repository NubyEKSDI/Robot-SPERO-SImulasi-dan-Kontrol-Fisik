from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bcr_bot',
            executable='lidar_bridge_node',
            name='lidar_bridge_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ''],  # Bisa diisi file rviz config jika ada
            output='screen',
        ),
    ]) 