from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_control',
            executable='mpc_controller',
            name='mpc_controller',
            output='screen'
        )
    ])