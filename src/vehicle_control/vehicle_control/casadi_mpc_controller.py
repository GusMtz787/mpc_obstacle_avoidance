# casadi_mpc_controller.py
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/mpc_predicted_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 20)
        self.pointCloud_sub = self.create_subscription(PointCloud2, '/obstacle_points', self.pointCloud_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialize state variable
        self.current_state = [0.0, 0.0, 0.0]  # Initial state: [x, y, theta]
        
        # Waypoints
        self.waypoints = [
            [11.0, 0.0, 0.0]
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
        T = 0.2        # sampling time [s]
        r_safe = 0.5   # safety radius around obstacle

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
        lbg = []            # numeric lower bounds
        ubg = []            # numeric upper bounds

        # --------------- initial condition --------------------
        g_ic = X[:, 0] - P[0:3]
        g_sym.append(g_ic)
        lbg.extend([0.0, 0.0, 0.0])
        ubg.extend([0.0, 0.0, 0.0])

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
            obj += ca.mtimes([pos_err.T, Q[0:2, 0:2], pos_err])
            obj += ca.mtimes([control_input.T, R, control_input])

            # dynamics constraint -----------------------------
            g_dyn = X[:, k + 1] - (current_state + T * f(current_state, control_input))
            g_sym.append(g_dyn)
            lbg.extend([0.0, 0.0, 0.0])
            ubg.extend([0.0, 0.0, 0.0])

            # obstacle constraint -----------------------------
            if closest_obs_ca is not None:
                dist2 = ca.sumsqr(current_state[0:2] - closest_obs_ca)
                g_obs = dist2 - r_safe ** 2  # >= 0 keeps us outside
                g_sym.append(g_obs)
                lbg.append(0.0)
                ubg.append(np.inf)

        # --------------- assemble NLP -------------------------
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        G = ca.vertcat(*g_sym)
        lbg = np.array(lbg, dtype=float)
        ubg = np.array(ubg, dtype=float)

        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': G, 'p': P}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob,
                           {'ipopt.print_level': 0, 'print_time': 0})

        # --------------- variable bounds ----------------------
        lbx = np.concatenate([
            np.full(((N + 1) * 3,), -np.inf),  # states
            np.tile([-1.0, -0.5], N)           # controls
        ])
        ubx = np.concatenate([
            np.full(((N + 1) * 3,), np.inf),   # states
            np.tile([1.0, 0.5], N)             # controls
        ])

        # --------------- initial guess & parameters ----------
        p = np.concatenate((x0, x_ref[:2]))
        x0_guess = np.tile(x0, (N + 1, 1)).T
        u0_guess = np.zeros((2, N))
        x_u_guess = np.concatenate((x0_guess.flatten(), u0_guess.flatten()))

        # --------------- solve -------------------------------
        sol = solver(x0=x_u_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

        # --------------- extract -----------------------------
        start_index = n_states * (N + 1)
        u_opt = sol['x'][start_index:start_index + 2]

        predicted_states = np.array(sol['x'][:n_states * (N + 1)].full()).reshape((N + 1, n_states))

        return float(u_opt[0]), float(u_opt[1]), predicted_states, solver.stats()

    # def solve_mpc(self, x0, x_ref):
        
    #     # General MPC setup
    #     N = 15  # prediction horizon
    #     T = 0.2  # time step
    #     min_safe_distance = 0.25  # minimum allowed distance to any obstacle

    #     # System variables
    #     x = ca.MX.sym('x')
    #     y = ca.MX.sym('y')
    #     theta = ca.MX.sym('theta')
    #     states = ca.vertcat(x, y, theta)
    #     n_states = states.size()[0]

    #     # Control variables
    #     v = ca.MX.sym('v')
    #     omega = ca.MX.sym('omega')
    #     controls = ca.vertcat(v, omega)
    #     n_controls = controls.size()[0]

    #     # Define the system dynamics
    #     # dx/dt = v * cos(theta)
    #     # dy/dt = v * sin(theta)
    #     # dtheta/dt = omega
    #     dynamics = ca.vertcat(
    #         v * ca.cos(theta),
    #         v * ca.sin(theta),
    #         omega
    #     )
    
    #     # Create a CasADi function for the dynamics
    #     f = ca.Function('f', [states, controls], [dynamics])

    #     # Define the optimization variables
    #     # These contain the full state trajectory and control inputs
    #     # X = [x0, x1, ..., xN], U = [u0, u1, ..., uN-1]
    #     # where xi is the state at time step i and ui is the control input at time step i
    #     # P is the initial state (x0) but in CasADi format they usually use parameter P for this purpose
    #     # We will optimize over the trajectory X and control inputs U
    #     X = ca.MX.sym('X', n_states, N+1)
    #     U = ca.MX.sym('U', n_controls, N)
    #     P = ca.MX.sym('P', n_states + 2)  # x0 and x_ref (but just 2D)

    #     # Weighting matrices for the cost function
    #     Q = ca.diag([2.0, 2.0, 1.0])
    #     R = ca.diag([0.1, 0.4])

    #     # Objective function and constraints
    #     # The objective is to minimize the distance to the reference trajectory and the control effort
    #     # g will hold the constraints, including the initial condition and dynamics constraints
    #     obj = 0
    #     g = []
    #     g.append(X[:, 0] - P[0:3])# initial condition constraint this tells that X[0] = P, where P is the initial state (x0). Bear in mind that g must then yield 0.

    #     if self.isEnable_obstacle_avoidance:
    #         if hasattr(self, 'obstacle_points') and len(self.obstacle_points) > 0:
    #             closest_obs = min(self.obstacle_points, key=lambda pt: np.linalg.norm(x0[:2] - pt))
    #             closest_obs_ca = ca.DM(closest_obs) # ca.DM defines a dense matrix in CasADi format
    #         else:
    #             closest_obs_ca = None

    #     # This is the core of the MPC loop
    #     # We iterate over the prediction horizon N
    #     # For each step, we compute the cost and the dynamics constraints
    #     # The cost function is quadratic in the state and control inputs
    #     # The dynamics constraints ensure that the predicted next state matches the actual next state
    #     for k in range(N):
    #         current_state = X[:, k]

    #         if self.isEnable_obstacle_avoidance and closest_obs_ca is not None:
    #             dist = ca.norm_2(current_state[0:2] - closest_obs_ca)
    #             penalty = ca.fmax(0, min_safe_distance - dist)
    #             g.append(dist - min_safe_distance)

    #         control_input = U[:, k]
    #         pos_error = current_state[0:2] - x_ref[0:2]
    #         obj += ca.mtimes([pos_error.T, Q[0:2, 0:2], pos_error])
    #         obj += ca.mtimes([control_input.T, R, control_input])
    #         next_state = X[:, k+1]
    #         prediction_next_state = current_state + T * f(current_state, control_input)
    #         g.append(next_state - prediction_next_state)

    #     # Now the optimization variables need to be reshaped and concatenated
    #     # CasADi requires the optimization variables to be in a specific format
    #     # We will reshape X and U to be column vectors and concatenate them
    #     # This is necessary for the solver to understand the problem structure
    #     # print("X shape: ", X.shape)
    #     # print("X shape reshaped: ", (ca.reshape(X, -1, 1)).shape)
    #     # print("U shape: ", ca.reshape(U, -1, 1).shape)
    #     # print("U shape reshaped: ", (ca.reshape(U, -1, 1)).shape)
    #     # print("X and U concatenated for Casadi: ", (ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))).shape)
    #     OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

    #     # Create the NLP problem
    #     # The objective function is the cost we defined above
    #     # The constraints are the dynamics constraints and the initial condition constraint
    #     nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}# use the * operator to unpack the list of constraints

    #     opts = {'ipopt.print_level': 0, 'print_time': 0}
    #     solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    #     # print("NLP constraints (g):", nlp_prob['g'].shape) # To verify the number of constraints

    #     # Set bounds for the optimization variables
    #     # The bound for the velocity v and angular velocity omega are set to [-1, 1]
    #     # The bounds for the states are set to [-inf, inf] for x, y, theta
    #     lower_bound_state_control = np.concatenate([
    #         np.full(((N+1) * 3,), -np.inf),   # State lower bounds: x, y, theta
    #         np.tile([-1.0, -0.5], N)          # Control lower bounds: v, omega
    #     ])
    #     upper_bound_state_control = np.concatenate([
    #         np.full(((N+1) * 3,),  np.inf),   # State upper bounds, numpy full fills an array with a given constant value
    #         np.tile([ 1.0,  0.5], N)          # Control upper bounds, tile repeats the array (pattern) to match the number of control inputs
    #     ])

    #     # Set the bounds for the inequality constraints, which are the dynamics constraints.
    #     # We set them to 0 because we want the predicted next state to match the actual next state
    #     # The constraints g should yield 0, so we set the lower and upper bounds to 0
    #     lower_bound_constraint = np.zeros((N+1) * n_states)
    #     upper_bound_constraint = np.zeros((N+1) * n_states)

    #     # Add the upper and lower bounds for the obstacle avoidance constraints:
    #     # if we do have an obstacle, we need the upper and lower bounds for all
    #     # the control horizon N. Else, we don't have any constraints.
    #     if self.isEnable_obstacle_avoidance and closest_obs_ca is not None:
    #         lower_bound_constraint = np.concatenate([lower_bound_constraint, np.full((N,), 0.0)])
    #         upper_bound_constraint = np.concatenate([upper_bound_constraint, np.full((N,), np.inf)])

    #     # Set the initial state as a parameter for the solver
    #     x_target = np.array(x_ref[:2])
    #     p = np.concatenate((x0, x_target))

    #     x0_inits = np.tile(x0, (N+1, 1)).T
    #     u0_inits = np.zeros((2, N))
    #     x_u_inits = np.concatenate((x0_inits.flatten(), u0_inits.flatten()))

    #     # Verify number of constraints
    #     self.get_logger().info(f"Constraints g: {ca.vertcat(*g).shape}")
    #     self.get_logger().info(f"lbg: {lower_bound_constraint.shape}, ubg: {upper_bound_constraint.shape}")

    #     # Solve the NLP problem
    #     sol = solver(x0=x_u_inits, lbx=lower_bound_state_control, ubx=upper_bound_state_control, lbg=lower_bound_constraint, ubg=upper_bound_constraint, p=p)

    #     # Extract the optimal control inputs from the solution which has the 
    #     # format [x0, y0, theta0, ..., xN, yN, thetaN, v0, omega0, ..., vN-1, omegaN-1]
    #     start_index = n_states * (N + 1)
    #     u_opt = sol['x'][start_index: start_index + 2]
        
    #     # Extract predicted trajectory
    #     predicted_states = sol['x'][:n_states * (N+1)]
    #     predicted_states = np.array(predicted_states.full()).reshape((N+1, n_states))

    #     return float(u_opt[0]), float(u_opt[1]), predicted_states, solver.stats()

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
