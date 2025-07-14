from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('bcr_bot')
    
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim = LaunchConfiguration('use_sim')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Set default values
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_sim',
        default_value='True',
        description='Whether to start Gazebo'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'gedung_p.sdf'),
        description='Full path to world model file to load'
    )

    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_dir, 'urdf', 'spero.urdf.xacro'),
        description='Full path to robot urdf file'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='True',
        description='Use ros2_control if true'
    )

    # Specify the launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        condition=IfCondition(use_sim),
        launch_arguments={'headless': headless}.items()
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('bcr_bot'), 'launch', 'rsp.launch.py')]),
        condition=IfCondition(use_robot_state_pub)
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('bcr_bot'), 'launch', 'rviz.launch.py')]),
        condition=IfCondition(use_rviz)
    )

    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'spero',
                  '-file', model],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_dir, 'config', 'spero_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_cmd,
        declare_simulator_cmd,
        declare_world_cmd,
        declare_model_cmd,
        declare_rviz_config_file_cmd,
        declare_use_ros2_control_cmd,
        gazebo,
        robot_state_publisher,
        rviz,
        spawn_entity_cmd,
        bridge
    ]) 