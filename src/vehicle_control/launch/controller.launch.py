from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_control',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen'
        )
    ])
