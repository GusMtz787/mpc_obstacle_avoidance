# casadi_mpc_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import casadi as ca

class CasadiMPCController(Node):
    def __init__(self):
        super().__init__('casadi_mpc_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/obstacle_points', self.pc_callback, 10)

        self.obstacle_points = np.empty((0, 2))
        self.timer = self.create_timer(0.2, self.control_loop)

        self.current_state = [0.0, 0.0, 0.0]  # Initial state: [x, y, theta]

        # Waypoints to follow (simple path)
        self.waypoints = [
            [0.0, 0.0, 0.0],
            [7.0, 0.0, 0.0]
        ]
        
        self.current_wp_idx = 0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        self.current_state = [x, y, theta]

    def pc_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            points.append([p[0], p[1]])
        self.obstacle_points = np.array(points)

    def control_loop(self):
        if self.current_state is None:
            return

        x0 = np.array(self.current_state)
        x_ref = np.array(self.waypoints[self.current_wp_idx])

        if np.linalg.norm(x0[:2] - x_ref[:2]) < 0.5 and self.current_wp_idx < len(self.waypoints) - 1:
            self.current_wp_idx += 1
            x_ref = np.array(self.waypoints[self.current_wp_idx])

        if self.current_wp_idx == len(self.waypoints) - 1 and np.linalg.norm(x0[:2] - x_ref[:2]) < 0.3:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Goal reached. Stopping.")
            return

        u_opt = self.solve_mpc(x0, x_ref)
        msg = Twist()
        msg.linear.x = float(u_opt[0])
        msg.angular.z = float(u_opt[1])
        self.cmd_pub.publish(msg)

    def solve_mpc(self, x0, x_ref):
        T = 0.2
        N = 15
        r_safe = 0.6

        x = ca.MX.sym('x', 3, N+1)
        u = ca.MX.sym('u', 2, N)

        cost = 0
        g = []
        g.append(x[:, 0] - x0)

        Q = np.diag([2, 2, 0.1])
        R = np.diag([0.1, 0.1])

        for k in range(N):
            cost += ca.mtimes([(x[:, k] - x_ref).T, Q, (x[:, k] - x_ref)])
            cost += ca.mtimes([u[:, k].T, R, u[:, k]])

            x_k = x[:, k]
            u_k = u[:, k]

            theta = x_k[2]
            next_x = x_k[0] + T * u_k[0] * ca.cos(theta)
            next_y = x_k[1] + T * u_k[0] * ca.sin(theta)
            next_theta = x_k[2] + T * u_k[1]

            g.append(x[:, k+1] - ca.vertcat(next_x, next_y, next_theta))

            # Obstacle avoidance constraint
            for p in self.obstacle_points:
                g.append(ca.norm_2(x[:2, k] - p) - r_safe)

        cost += ca.mtimes([(x[:, N] - x_ref).T, Q, (x[:, N] - x_ref)])

        vars = ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1))
        nlp = {'x': vars, 'f': cost, 'g': ca.vertcat(*g)}
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-2}

        solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        lbg = [0]*len(g)  # all equality + lower bounds for obstacle constraint
        ubg = [0]*len(g)
        for i in range(N):
            for _ in self.obstacle_points:
                ubg[i+1] = ca.inf  # allow distance to be larger than r_safe

        sol = solver(x0=ca.DM.zeros(vars.shape), lbg=lbg, ubg=ubg)

        u_opt = ca.reshape(sol['x'][3*(N+1):], 2, N)
        return u_opt[:, 0].full().flatten()


def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
