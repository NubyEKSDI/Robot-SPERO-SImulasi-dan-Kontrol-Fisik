from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bcr_bot'

# Function to get all files recursively in a directory
def get_data_files(directory):
    data_files = []
    for root, dirs, files in os.walk(directory):
        if files:
            relative_root = root
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((os.path.join('share', package_name, relative_root), file_list))
    return data_files

# Get all model data files recursively
model_data_files = get_data_files('models')
mesh_data_files = get_data_files('meshes')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/bcr_bot/launch', glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds', 'gedung_p'), glob('worlds/gedung_p/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ] + model_data_files + mesh_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuby',
    maintainer_email='nuby@todo.todo',
    description='BCR Bot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roam = scripts.roam:main',
            'remapper = scripts.remapper:main',
            'initial_pose_publisher = scripts.initial_pose_publisher:main',
            'lidar_bridge_node = scripts.lidar_bridge_node:main',
            'mqtt_to_ros2_bridge = scripts.mqtt_to_ros2_bridge:main',
            'global_mapping_node = scripts.global_mapping_node:main',
            'robot_sync_bridge = scripts.robot_sync_bridge:main',
        ],
    },
) 