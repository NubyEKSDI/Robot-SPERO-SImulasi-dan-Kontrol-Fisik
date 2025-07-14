from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('bcr_bot')
    
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    # Process XACRO file
    xacro_file = os.path.join(pkg_dir, 'urdf', 'spero.urdf.xacro')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'spero.urdf')
    
    # Convert XACRO to URDF
    xacro_process = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file, 'r').read()
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher'
        ),
        xacro_process,
        robot_state_publisher
    ]) 