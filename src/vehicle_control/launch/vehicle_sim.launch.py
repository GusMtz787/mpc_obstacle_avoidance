from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('vehicle_control')

    return LaunchDescription([
        # World
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'world.launch.py')
            )
        ),

        # Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'controller.launch.py')
            )
        ),

        # Lidar listener
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'lidar_listener.launch.py')
            )
        ),
    ])
