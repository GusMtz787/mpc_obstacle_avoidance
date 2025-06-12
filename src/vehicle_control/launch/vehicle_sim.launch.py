from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to the Gazebo world file
    world_path = os.path.join(
        get_package_share_directory('vehicle_control'),
        'worlds',
        'diff_robot.sdf'
    )

    return LaunchDescription([
        
        # 1. We launch Gazebo Sim with your world
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path],
            output='screen'
        ),

        # 2. Now we need to bridge the lidar and cmd_vel topics between ROS 2 and Ignition
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
            output='screen'
        ),

        # 3. We run the vehicle controller
        Node(
            package='vehicle_control',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen'
        ),

        # 4. We can also test the lidar by creating a listener node
        Node(
            package='vehicle_control',
            executable='lidar_listener',
            name='lidar_listener',
            output='screen'
        ),
    ])