from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
            output='screen'
        )
    ])
