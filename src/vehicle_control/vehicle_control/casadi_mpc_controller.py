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
        self.odom_sub = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 20)
        self.pointCloud_sub = self.create_subscription(PointCloud2, '/obstacle_points', self.pointCloud_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_state = [0.0, 0.0, 0.0]  # Initial state: [x, y, theta]

        # Waypoints to follow (simple path)
        self.waypoints = [
            [0.0, 0.0, 0.0],
            [3.0, 2.0, 0.0],
            [4.0, 3.0, 0.0],
            [5.0, 5.0, 0.0]
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

    def pointCloud_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            points.append([p[0], p[1]])
        self.obstacle_points = np.array(points)

    def control_loop(self):

        x0 = np.array(self.current_state)
        x_ref = np.array(self.waypoints[self.current_wp_idx])

        if np.linalg.norm(x0[:2] - x_ref[:2]) < 0.5 and self.current_wp_idx < len(self.waypoints) - 1:
            self.current_wp_idx += 1
            x_ref = np.array(self.waypoints[self.current_wp_idx])

        if self.current_wp_idx == len(self.waypoints) - 1 and np.linalg.norm(x0[:2] - x_ref[:2]) < 0.3:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Goal reached. Stopping.")
            return

        self.get_logger().info(f'Current state: {x0}, Target waypoint: {x_ref}')
        v, omega = self.solve_mpc(x0, x_ref)

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent cmd_vel: v={v:.2f}, omega={omega:.2f}')

    def solve_mpc(self, x0, x_ref):
        
        # General MPC setup
        N = 15  # prediction horizon
        T = 0.2  # time step

        # System variables
        x = ca.MX.sym('x')
        y = ca.MX.sym('y')
        theta = ca.MX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.size()[0]

        # Control variables
        v = ca.MX.sym('v')
        omega = ca.MX.sym('omega')
        controls = ca.vertcat(v, omega)
        n_controls = controls.size()[0]

        # Define the system dynamics
        # dx/dt = v * cos(theta)
        # dy/dt = v * sin(theta)
        # dtheta/dt = omega
        dynamics = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            omega
        )
    
        # Create a CasADi function for the dynamics
        f = ca.Function('f', [states, controls], [dynamics])

        # Define the optimization variables
        # These contain the full state trajectory and control inputs
        # X = [x0, x1, ..., xN], U = [u0, u1, ..., uN-1]
        # where xi is the state at time step i and ui is the control input at time step i
        # P is the initial state (x0) but in CasADi format they usually use parameter P for this purpose
        # We will optimize over the trajectory X and control inputs U
        X = ca.MX.sym('X', n_states, N+1)
        U = ca.MX.sym('U', n_controls, N)
        P = ca.MX.sym('P', n_states + 2)  # x0 and x_ref (but just 2D)

        # Weighting matrices for the cost function
        Q = ca.diag([2.0, 2.0, 1.0])
        R = ca.diag([0.1, 0.4])

        # Objective function and constraints
        # The objective is to minimize the distance to the reference trajectory and the control effort
        # g will hold the constraints, including the initial condition and dynamics constraints
        obj = 0
        g = []
        g.append(X[:, 0] - P[0:3])# initial condition constraint this tells that X[0] = P, where P is the initial state (x0). Bear in mind that g must then yield 0.

        # This is the core of the MPC loop
        # We iterate over the prediction horizon N
        # For each step, we compute the cost and the dynamics constraints
        # The cost function is quadratic in the state and control inputs
        # The dynamics constraints ensure that the predicted next state matches the actual next state
        for k in range(N):
            current_state = X[:, k]
            control_input = U[:, k]
            pos_error = current_state[0:2] - x_ref[0:2]
            obj += ca.mtimes([pos_error.T, Q[0:2, 0:2], pos_error])
            obj += ca.mtimes([control_input.T, R, control_input])
            next_state = X[:, k+1]
            prediction_next_state = current_state + T * f(current_state, control_input)
            g.append(next_state - prediction_next_state)

        # Now the optimization variables need to be reshaped and concatenated
        # CasADi requires the optimization variables to be in a specific format
        # We will reshape X and U to be column vectors and concatenate them
        # This is necessary for the solver to understand the problem structure
        # print("X shape: ", X.shape)
        # print("X shape reshaped: ", (ca.reshape(X, -1, 1)).shape)
        # print("U shape: ", ca.reshape(U, -1, 1).shape)
        # print("U shape reshaped: ", (ca.reshape(U, -1, 1)).shape)
        # print("X and U concatenated for Casadi: ", (ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))).shape)
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

        # Create the NLP problem
        # The objective function is the cost we defined above
        # The constraints are the dynamics constraints and the initial condition constraint
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}# use the * operator to unpack the list of constraints

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # print("NLP constraints (g):", nlp_prob['g'].shape) # To verify the number of constraints

        # Set bounds for the optimization variables
        # The bound for the velocity v and angular velocity omega are set to [-1, 1]
        # The bounds for the states are set to [-inf, inf] for x, y, theta
        ubx = np.concatenate([
            np.full(((N+1) * 3,),  np.inf),   # State upper bounds, numpy full fills an array with a given constant value
            np.tile([ 1.0,  0.5], N)          # Control upper bounds, tile repeats the array (pattern) to match the number of control inputs
        ])
        lbx = np.concatenate([
            np.full(((N+1) * 3,), -np.inf),   # State lower bounds: x, y, theta
            np.tile([-1.0, -0.5], N)          # Control lower bounds: v, omega
        ])

        # Set the bounds for the inequality constraints, which are the dynamics constraints.
        # We set them to 0 because we want the predicted next state to match the actual next state
        # The constraints g should yield 0, so we set the lower and upper bounds to 0
        lbg = np.zeros((N+1) * n_states)
        ubg = np.zeros((N+1) * n_states)

        # Set the initial state as a parameter for the solver
        x_target = np.array(x_ref[:2])
        p = np.concatenate((x0, x_target))

        x0_inits = np.tile(x0, (N+1, 1)).T
        u0_inits = np.zeros((2, N))
        x_u_guess = np.concatenate((x0_inits.flatten(), u0_inits.flatten()))

        # Solve the NLP problem
        sol = solver(x0=x_u_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

        # Extract the optimal control inputs from the solution which has the 
        # format [x0, y0, theta0, ..., xN, yN, thetaN, v0, omega0, ..., vN-1, omegaN-1]
        start_index = n_states * (N + 1)
        u_opt = sol['x'][start_index: start_index + 2]
        
        return float(u_opt[0]), float(u_opt[1])

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
