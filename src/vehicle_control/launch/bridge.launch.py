from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
            ],
            output='screen'
        )
    ])
