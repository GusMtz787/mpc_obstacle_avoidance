import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_forward)
        self.lidar_sub = self.create_subscription(LaserScan, 
                                                  '/lidar', 
                                                  self.lidar_callback, 
                                                  10) # Subscribe to the LIDAR topic
        self.obstacle_points = np.empty((0, 2)) # Initialize an empty array for obstacle points
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10) # Publish obstacle points as PointCloud2 for visualization

    # Callback for LiDAR data, this function processes the incoming LaserScan messages
    # and extracts the obstacle points in front of the robot.
    #
    # It filters out invalid ranges, converts polar coordinates to Cartesian coordinates,
    # and publishes the points as a PointCloud2 message.
    def lidar_callback(self, msg):
        # Extract ranges and angles from the LaserScan message
        ranges = np.array(msg.ranges)

        # Calculate angles based on the message data:
        # angle_min is the angle of the first range, angle_increment is the angle between each range
        # This gives us the angles corresponding to each range measurement.
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filter valid points: 
        # - Ignore ranges that are too small (in this case it is configured to less than 0.1)
        # - Ignore ranges that exceed the maximum range of the LiDAR sensor which is specified in the message.
        # This ensures we only keep meaningful measurements.
        # The valid points are then used to filter the ranges and angles by a boolean mask.
        valid = (ranges > 0.1) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        # Polar coordinates (ranges, angles) are converted to Cartesian coordinates (x, y)
        # using the formulas:
        # x = r * cos(theta)
        # y = r * sin(theta)
        # where r is the range and theta is the angle.
        # This gives us the (x, y) coordinates of the points in front of the robot.
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # Stack the x and y coordinates into a 2D array of points
        # The points are structured as (x, y) pairs. For example:
        # [[x1, y1],
        #  [x2, y2]]
        points = np.vstack((xs, ys)).T

        # Filter points to keep only those in front of the robot:
        # - x > 0.0: This ensures we only keep points in front of the robot (assuming the robot is facing the positive x direction).
        # - |y| < 3.0: This keeps points within a certain lateral distance from the robot and discarding points that are too far to the left or right.
        forward_filter = (points[:, 0] > 0.0) & (np.abs(points[:, 1]) < 3.0)
        self.obstacle_points = points[forward_filter]

        # Publish the obstacle points as PointCloud2
        if self.obstacle_points.size > 0:
            self.publish_obstacle_points(msg)

    # This function publishes the obstacle points as a PointCloud2 message.
    # It converts the 2D points into a 3D format (with z=0) and uses the header from the incoming message
    # to maintain the timestamp and frame of reference.
    # The PointCloud2 message is then published to the '/obstacle_points' topic for visualization.
    def publish_obstacle_points(self, msg):
        points_3d = []
        
        # Create 3D points (z=0)
        for x, y in self.obstacle_points:
            point = [float(x), float(y), 0.0]
            points_3d.append(point)

        # Create header
        header = msg.header  # reuse the lidar timestamp + frame

        # Convert to PointCloud2
        cloud_msg = pc2.create_cloud_xyz32(header, points_3d)

        # Publish
        self.pointcloud_pub.publish(cloud_msg)

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
