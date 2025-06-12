from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('vehicle_control'),
        'worlds',
        'diff_robot.sdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path],
            output='screen'
        )
    ])
