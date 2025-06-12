import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.sub = self.create_subscription(
            LaserScan, '/lidar', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Min range: {min(msg.ranges):.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
