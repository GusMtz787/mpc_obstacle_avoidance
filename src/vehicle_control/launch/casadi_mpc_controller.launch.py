from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'casadi_mpc_params.yaml'
    )

    print("Config path:", config)

    return LaunchDescription([
        Node(
            package='vehicle_control',
            executable='casadi_mpc_controller',
            name='casadi_mpc_controller',
            output='screen',
            parameters=[config],
            arguments=['--ros-args', '--params-file', config]
        )
    ])