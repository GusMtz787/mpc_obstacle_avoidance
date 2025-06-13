import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import cvxpy as cp

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 20)
        self.timer = self.create_timer(0.2, self.control_loop)

        self.current_state = [0.0, 0.0, 0.0]  # x, y, theta
        self.target_state = [5.0, 5.0, 0.0]   # desired x, y, theta

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        # The quaternion is represented as (w, x, y, z),
        # where w is the scalar part and (x, y, z) are the vector part.
        # The yaw angle (theta) can be computed from the quaternion using:
        # theta = arctan(y/x) = atan2(y-coordinate, x-coordinate) = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        # This gives us the orientation of the robot in the plane.
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)

        # Update the current state with the latest odometry data
        self.current_state = [x, y, theta]
        self.get_logger().info(f'Current state: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

    # This function solves the MPC optimization problem to compute the control inputs
    # (linear velocity and angular velocity) that will drive the robot towards the target state.
    #
    # For this we use the cvxpy library to formulate and solve a quadratic programming problem.
    # The problem minimizes a cost function that penalizes deviations from the reference state
    # and the control inputs, which is subject to system dynamics and input constraints.
    #
    # Parameters:
    # - x0: Current state of the robot (x, y, theta).
    # - x_ref: Reference state to be achieved (target x, target y, theta).
    #
    # Returns:
    # - A tuple (v, omega) representing the linear and angular velocities.
    #   If the optimization fails, it returns (0.0, 0.0) as a fallback.
    # 
    # Note: The MPC formulation assumes a simple kinematic model of the robot.
    # The state is represented as a vector [x, y, theta], and the control inputs are [v, omega].
    # The robot's motion is modeled using a linearized state-space representation.
    #
    # The cost function is defined as:
    # J = sum(Q * (x_k - x_ref)^2 + R * u_k^2) for k = 0 to N-1
    # where Q and R are weight matrices for state and control input penalties, respectively.
    # The constraints ensure that the state evolves according to the linearized dynamics:
    # x_{k+1} = A * x_k + B * u_k, where A and B are the system matrices.
    #
    # The control inputs are also constrained to be within specified limits.
    def solve_mpc(self, x0, x_ref):
        T = 0.2  # sampling time
        N = 15   # horizon

        Q = np.diag([1, 1, 1]) # State cost matrix
        R = np.diag([0.1, 0.1]) # Control cost matrix

        x = cp.Variable((3, N + 1)) # State variable (x, y, theta) for each time step, cvxpy.Variable creates a variable that will be optimized
        u = cp.Variable((2, N)) # Control input variable (v, omega) for each time step

        theta0 = x0[2] # Initial orientation of the robot

        # Linearized A, B matrices
        A = np.eye(3)
        A[0, 2] = -T * np.sin(theta0)
        A[1, 2] =  T * np.cos(theta0)

        B = np.array([
            [T * np.cos(theta0), 0],
            [T * np.sin(theta0), 0],
            [0, T]
        ])

        # Setting up the optimization problem
        # First initialize the cost and constraints
        cost = 0
        constraints = [x[:, 0] == x0] # Initial state constraint
        v_min, v_max = -1.0, 1.0 # Linear velocity limits
        omega_min, omega_max = -1.0, 1.0 # Angular velocity limits

        # Loop through the prediction horizon
        # For each time step, we add the cost and constraints
        for k in range(N):
            cost += cp.quad_form(x[:, k] - x_ref, Q) + cp.quad_form(u[:, k], R)
            constraints += [x[:, k + 1] == A @ x[:, k] + B @ u[:, k]]
            constraints += [
            u[0, k] >= v_min,
            u[0, k] <= v_max,
            u[1, k] >= omega_min,
            u[1, k] <= omega_max
            ]

        cost += cp.quad_form(x[:, N] - x_ref, Q)

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        if u.value is not None:
            return u[:, 0].value
        else:
            return [0.0, 0.0]

    # This function is called periodically to compute and send the control commands
    # to the robot based on the current state and the target state.
    # It uses the MPC solver to compute the optimal control inputs (linear and angular velocities)
    # and publishes them as a Twist message to the '/cmd_vel' topic.
    def control_loop(self):
        v, omega = self.solve_mpc(self.current_state, self.target_state)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent cmd_vel: v={v:.2f}, omega={omega:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
