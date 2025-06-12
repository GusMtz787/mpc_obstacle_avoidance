import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.2
        self.cmd_pub.publish(msg)
        self.get_logger().info('Sending cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
