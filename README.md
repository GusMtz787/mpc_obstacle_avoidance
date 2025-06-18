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
├── config/
│ ├── casadi_mpc_params.yaml # configurable parameters
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

### Run the project

```bash
# In console 1 run: 
ros2 launch vehicle_control bridge.launch.py 

# In console 2 run:
ros2 launch vehicle_control lidar_listener.launch.py

# In console 3 run:
ros2 launch vehicle_control world.launch.py

# In the last console run:
ros2 launch vehicle_control casadi_mpc_controller.launch.py
```

---

## Mathematical Formulation

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

- $x_k = [x_k, y_k]^T$
- $u_k = [v_k, \omega_k]^T$
- $Q_p = \text{diag}(2.0, 2.0)$
- $R = \text{diag}(0.1, 0.4)$
- $v_{\min} = -1.0,\; v_{\max} = 1.0$
- $\omega_{\min} = -0.5,\; \omega_{\max} = 0.5$
- $(x_{obs}, y_{obs})$: coordinates of closest detected obstacle
- $r_{\text{safe}} = 2.0$ meters (safety margin)

> NOTE: Obstacle avoidance is enforced using a **distance-squared constraint** to the closest detected point from a LiDAR-derived point cloud. This law has been successfully tested in:
[MPC-Based Obstacle Avoidance Path Tracking Control for Distributed Drive Electric Vehicles](https://doi.org/10.3390/wevj13120221), *Wu, H., Zhang, H., and Feng, Y.*, *World Electric Vehicle Journal, 2022*.

## Checklist: MPC Obstacle Avoidance

| Feature / Requirement                          | Status | Notes                                                      |
|------------------------------------------------|--------|------------------------------------------------------------|
| **ROS 2 Humble Compatibility**                 | Done   | Uses `rclpy`, ROS 2 launch files, and parameters           |
| **Gazebo Sim Integration**                     | Done   | Differential-drive robot with LiDAR in simulated world     |
| **Model Predictive Control (CVXPY)**           | Done   | Implemented in `mpc_controller.py` using CVXPY             |
| **Nonlinear Model Predictive Control (CasADi)**| Done   | Implemented in `casadi_mpc_controller.py` using CasADi     |
| **Path Following**                             | Done   | Robot tracks waypoints using MPC-based control             |
| **LiDAR Integration**                          | Done   | Converts `LaserScan` to `PointCloud2` for MPC              |
| **Configurable Cost Weights**                  | Done   | `path_weight` and `obstacle_weight` via YAML parameters    |
| **Parameter YAML Support**                     | Done   | Uses `casadi_mpc_params.yaml` loaded at runtime            |
| **Visualization**                              | Done   | Publishes `Path` and `Marker` topics for RViz/Gazebo       |
| **Obstacle Avoidance**                         | Pending| Currently the odometry drifts over time                    |

## Known Issues:

The only identified issue with the current set-up, is the plugin used for this car. As it can be seen from the .sdf set-up, the odometry is being pulled from the wheels. That means, for direct routes, the controller works without flaws because the odometry drift is minimal, if any. However, when considering obstacles, the corrections will make the odometry drift gradually. This is seen specially if one visualizes the `visualization_marker` topic in rviz2, the position of the car begins to drift when compared to the actual position. This behavior hinders the obstacle avoidance problem for the moment.

**Hypothesis:** feeding the pose of the car directly from Gazebo, instead of the wheels (discarding some of the reality of the problem by considering the odometry directly from the wheels), may allow the MPC drive the car to the desired position with obstacle avoidance.
