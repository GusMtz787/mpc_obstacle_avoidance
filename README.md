# MPC Obstacle Avoidance (ROS 2 + Gazebo Sim)

This ROS 2 project implements a **Model Predictive Control (MPC)** based navigation and obstacle avoidance system for a differential drive robot simulated in **Gazebo Sim**. The robot follows a given path while dynamically avoiding obstacles detected via LiDAR.

## Project Features

- **Differential Drive Vehicle** simulated in Gazebo.
- **Path Tracking** via two MPC implementations:
  - `cvxpy`-based (`mpc_controller.py`)
  - `CasADi`-based (`casadi_mpc_controller.py`)
- **Obstacle Avoidance** with live LiDAR data processing (pending).
- ROS 2 communication using `sensor_msgs`, `nav_msgs`, and `geometry_msgs`
- Visualization of planned trajectories and obstacle markers in RViz or Gazebo

---

## Repository Structure

```bash
vehicle_control/
├── launch/
│ ├── bridge.launch.py # Gazebo <-> ROS 2 bridge
│ ├── casadi_mpc_controller.launch.py # Casadi MPC implementation
│ ├── controller.launch.py # Basic motion controller
│ ├── lidar_listener.launch.py
│ ├── mpc_controller.launch.py # CVXPY MPC implementation
│ ├── vehicle_sim.launch.py # Launches full sim stack (with cvxpy)
│ └── world.launch.py # Launches Gazebo world
├── worlds/
│ └── diff_robot.sdf # Robot and environment description
├── mpc_controller.py # cvxpy-based MPC controller
├── casadi_mpc_controller.py # CasADi-based MPC with obstacle avoidance
├── vehicle_controller.py # Simple velocity command publisher
└── lidar_listener.py # LiDAR to PointCloud2 processing
```

---

## Start

### Prerequisites

- ROS 2 Humble
- Gazebo Fortress or compatible
- Python 3.10+
- Dependencies:

```bash
pip install casadi cvxpy numpy
```

### Build the ros2 workspace

```bash
cd ros2_workspace
colcon build
source install/setup.bash
```

---

## Mathematical Formulation (MPC)

The control problem in the `casadi_mpc_controller.py` file uses a discrete-time nonlinear Model Predictive Control (MPC) scheme to compute optimal velocity $u = [v \quad \omega]$ commands for a differential-drive robot over a prediction horizon \( N \), with obstacle avoidance.

### Dynamics for a differential-drive robot

The robot is modeled as a unicycle-like differential drive system with the following **continuous-time** kinematic equations:

$$
\begin{aligned}
\dot{x}(t) &= v(t) \cdot \cos(\theta(t)) \\
\dot{y}(t) &= v(t) \cdot \sin(\theta(t)) \\
\dot{\theta}(t) &= \omega(t)
\end{aligned}
$$

Where:

- $x(t), y(t)$: position of the robot in 2D
- $\theta(t)$: orientation (yaw angle)
- $v(t)$: linear velocity input
- $\omega(t)$: angular velocity input

Because the system is discretized, a forward Euler integration was selected to program the dynamics of the system.

$$
\begin{aligned}
x_{k+1} &= x_k + T \cdot v_k \cdot \cos(\theta_k) \\
y_{k+1} &= y_k + T \cdot v_k \cdot \sin(\theta_k) \\
\theta_{k+1} &= \theta_k + T \cdot \omega_k
\end{aligned}
$$

Where:

- $x_k, y_k, \theta_k$: state at step $k$.
- $v_k, \omega_k$: linear and angular velocity inputs.
- $T$: sampling time (in this simulation $T = 0.2$ s).

### Optimization Problem

At each timestep, the MPC solves the following optimization problem:

$$
\min_{v_k, \omega_k} = J
$$

By minimizing the cost function $J$, we can calculate the optimal values for $v$ and $w$. And the cost function is given by:

$$
J = \sum_{k=0}^{N-1} \left[(x_k - x_{\text{ref}})^T Q_p (x_k - x_{\text{ref}}) + u_k^T R u_k \right]
$$

subject to:

$$
x_0 = x_{\text{init}}
$$

$$
x_{k+1} = x_k + T \cdot f(x_k, u_k) \quad \forall k = 0,\dots,N-1
$$

$$
v_{\min} \leq v_k \leq v_{\max}
$$

$$
\omega_{\min} \leq \omega_k \leq \omega_{\max}
$$

$$
(x_k - x_{obs})^2 + (y_k - y_{obs})^2\geq r_{\text{safe}}^2
$$

Where:

- \( x_k = [x_k, y_k]^T \)
- \( u_k = [v_k, \omega_k]^T \)
- \( Q_p = \text{diag}(2.0, 2.0) \)
- \( R = \text{diag}(0.1, 0.4) \)
- \( v_{\min} = -1.0,\; v_{\max} = 1.0 \)
- \( \omega_{\min} = -0.5,\; \omega_{\max} = 0.5 \)
- \( p_{\text{obs}} \): closest detected obstacle
- \( r_{\text{safe}} = 0.5 \): safety margin

Obstacle avoidance is enforced using a **distance-squared constraint** to the closest detected point from a LiDAR-derived point cloud.