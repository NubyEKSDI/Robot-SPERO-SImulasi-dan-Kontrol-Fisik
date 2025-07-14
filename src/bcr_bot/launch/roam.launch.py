from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the roam.py script
    pkg_dir = get_package_share_directory('bcr_bot')
    script_path = os.path.join(pkg_dir, '..', '..', 'src', 'bcr_bot', 'scripts', 'roam.py')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['/usr/bin/python3', script_path],
            name='roaming_robot',
            output='screen'
        )
    ]) 