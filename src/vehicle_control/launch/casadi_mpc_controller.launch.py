from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_control',
            executable='casadi_mpc_controller',
            name='casadi_mpc_controller',
            output='screen'
        )
    ])