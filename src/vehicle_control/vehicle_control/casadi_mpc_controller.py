import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker

import numpy as np
import casadi as ca

class CasadiMPCController(Node):
    def __init__(self):
        super().__init__('casadi_mpc_controller')
        
        # Create parameters for path and obstacle weights (only if not set in YAML file)
        self.declare_parameter("path_weight", 1.0)
        self.declare_parameter("obstacle_weight", 0.5)

        # Retrieve the actual values (from YAML)
        self.path_weight = self.get_parameter("path_weight").value
        self.obstacle_weight = self.get_parameter("obstacle_weight").value

        self.get_logger().info(f"Using path_weight={self.path_weight}, obstacle_weight={self.obstacle_weight}")
        
        # Create publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/mpc_predicted_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 20)
        self.pointCloud_sub = self.create_subscription(PointCloud2, '/obstacle_points', self.pointCloud_callback, 10)
        
        # Create a timer to call the control loop at a fixed rate
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialize state variable
        self.current_state = [-11.0, 0.0, 0.0]  # Initial state: [x, y, theta]
        
        # Waypoints
        self.waypoints = [
            [11.0, 5.0, 0.0]
        ]
        
        # Waypoint index
        self.current_wp_idx = 0

        self.goal_reached = False
        
        self.distance_to_goal =  np.array(self.current_state[:2]) - np.array(self.waypoints[-1][:2])

        # Obstacle points
        self.obstacle_points = np.array([])

        # Enable obstacle avoidance
        self.isEnable_obstacle_avoidance = True 

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        self.current_state = [x, y, theta]

    def pointCloud_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            points.append([p[0], p[1]])
        self.obstacle_points = np.array(points)

    def control_loop(self):

        if self.current_state is None:
            return

        x0 = np.array(self.current_state)
        x_ref = np.array(self.waypoints[self.current_wp_idx])
        self.distance_to_goal = np.linalg.norm(x0[:2] - x_ref[:2])

        if self.distance_to_goal < 0.5 and self.current_wp_idx < len(self.waypoints) - 1:
            self.current_wp_idx += 1
            x_ref = np.array(self.waypoints[self.current_wp_idx])

        if self.current_wp_idx == len(self.waypoints) - 1:
            if not self.goal_reached:
                self.goal_reached = False  # initialize flag

            if self.distance_to_goal < 0.3 and not self.goal_reached:
                self.goal_reached = True
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'Current state: {x0}, Target waypoint: {x_ref}, Residual error: {self.distance_to_goal}')
                self.get_logger().info("Goal reached. Stopping.")
                return

            # prevent re-activating controller if already stopped
            if self.goal_reached:
                return

        v, omega, predicted_states, stats = self.solve_mpc(x0, x_ref)

        status = stats['success']
        iterations = stats['iter_count']

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Current state: {x0}, Target waypoint: {x_ref}, solver success: {status}, iterations: {iterations},sent cmd_vel: v={v:.2f}, omega={omega:.2f}')

        if hasattr(self, 'obstacle_points') and len(self.obstacle_points) > 0:
            closest_obs = min(self.obstacle_points, key=lambda pt: np.linalg.norm(x0[:2] - pt))
            
            marker = Marker()
            marker.header.frame_id = "vehicle_blue/chassis/gpu_lidar"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "closest_obstacle"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(closest_obs[0])
            marker.pose.position.y = float(closest_obs[1])
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

        car_marker = Marker()
        car_marker.header.frame_id = "vehicle_blue/chassis/gpu_lidar"
        car_marker.header.stamp = self.get_clock().now().to_msg()
        car_marker.ns = "car_position"
        car_marker.id = 1
        car_marker.type = Marker.CUBE
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = x0[0]
        car_marker.pose.position.y = x0[1]
        car_marker.pose.position.z = 0.1
        car_marker.scale.x = 0.3
        car_marker.scale.y = 0.2
        car_marker.scale.z = 0.2
        car_marker.color.r = 0.0
        car_marker.color.g = 0.0
        car_marker.color.b = 1.0
        car_marker.color.a = 1.0
        self.marker_pub.publish(car_marker)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "vehicle_blue/chassis/gpu_lidar"

        for state in predicted_states:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def solve_mpc(self, x0, x_ref):
        # --------------- problem parameters ----------------
        N = 15         # horizon length
        T = 0.1        # sampling time [s]
        r_safe = 2   # safety radius around obstacle

        # --------------- symbolic variables ---------------
        x, y, theta = ca.MX.sym('x'), ca.MX.sym('y'), ca.MX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.size1()

        v, omega = ca.MX.sym('v'), ca.MX.sym('omega')
        controls = ca.vertcat(v, omega)
        n_controls = controls.size1()

        f = ca.Function('f', [states, controls], [
            ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)
        ])

        # decision variables ------------------------------------------------
        X = ca.MX.sym('X', n_states, N + 1)  # predicted states
        U = ca.MX.sym('U', n_controls, N)    # control inputs
        P = ca.MX.sym('P', n_states + 2)     # parameters = x0 + x_ref(2)

        # --------------- cost weights & storage ----------------
        Q = ca.diag([2.0, 2.0, 1.0])
        R = ca.diag([0.1, 0.4])

        obj = 0
        g_sym = []          # symbolic constraints
        lower_bound_constraint = []            # numeric lower bounds
        upper_bound_constraint = []            # numeric upper bounds

        # --------------- initial condition --------------------
        g_ic = X[:, 0] - P[0:3]
        g_sym.append(g_ic)
        lower_bound_constraint.extend([0.0, 0.0, 0.0])
        upper_bound_constraint.extend([0.0, 0.0, 0.0])

        # pick the obstacle once per solve ---------------------
        closest_obs_ca = None
        if self.isEnable_obstacle_avoidance and len(self.obstacle_points) > 0:
            closest_obs = min(self.obstacle_points, key=lambda pt: np.linalg.norm(x0[:2] - pt))
            closest_obs_ca = ca.DM(closest_obs)

        # --------------- main Horizon loop --------------------
        for k in range(N):
            current_state = X[:, k]
            control_input = U[:, k]

            # stage cost -------------------------------------
            pos_err = current_state[0:2] - x_ref[0:2]
            obj += self.path_weight * ca.mtimes([pos_err.T, Q[0:2, 0:2], pos_err]) # only penalize position error, not orientation (for now)
            obj += ca.mtimes([control_input.T, R, control_input])

            # dynamics constraint -----------------------------
            g_dyn = X[:, k + 1] - (current_state + T * f(current_state, control_input))
            g_sym.append(g_dyn)
            lower_bound_constraint.extend([0.0, 0.0, 0.0])
            upper_bound_constraint.extend([0.0, 0.0, 0.0])

            # obstacle constraint -----------------------------
            if closest_obs_ca is not None:
                dist2 = ca.sumsqr(current_state[0:2] - closest_obs_ca)
                g_obs = self.obstacle_weight * (dist2 - r_safe ** 2)  # >= 0 this will ensure that the distance to the obstacle is at least r_safe
                g_sym.append(g_obs)
                lower_bound_constraint.append(0.0)
                upper_bound_constraint.append(np.inf)

        # --------------- assemble NLP -------------------------
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        G = ca.vertcat(*g_sym)
        lower_bound_constraint = np.array(lower_bound_constraint, dtype=float)
        upper_bound_constraint = np.array(upper_bound_constraint, dtype=float)

        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': G, 'p': P}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob,
                           {'ipopt.print_level': 0, 'print_time': 0})

        # --------------- variable bounds ----------------------
        lower_bound_state = np.concatenate([
            np.full(((N + 1) * 3,), -np.inf),  # states
            np.tile([-1.0, -0.5], N)           # controls
        ])
        upper_bound_state = np.concatenate([
            np.full(((N + 1) * 3,), np.inf),   # states
            np.tile([1.0, 0.5], N)             # controls
        ])

        # Verify number of constraints
        self.get_logger().info(f"Constraints g: {G.shape}")
        self.get_logger().info(f"lbg: {lower_bound_constraint.shape}, ubg: {upper_bound_constraint.shape}")

        # --------------- initial guess & parameters ----------
        p = np.concatenate((x0, x_ref[:2]))
        x0_guess = np.tile(x0, (N + 1, 1)).T
        u0_guess = np.zeros((2, N))
        x_u_guess = np.concatenate((x0_guess.flatten(), u0_guess.flatten()))

        # --------------- solve -------------------------------
        sol = solver(x0=x_u_guess, lbx=lower_bound_state, ubx=upper_bound_state, lbg=lower_bound_constraint, ubg=upper_bound_constraint, p=p)

        # --------------- extract -----------------------------
        start_index = n_states * (N + 1)
        u_opt = sol['x'][start_index:start_index + 2]

        predicted_states = np.array(sol['x'][:n_states * (N + 1)].full()).reshape((N + 1, n_states))

        return float(u_opt[0]), float(u_opt[1]), predicted_states, solver.stats()

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
