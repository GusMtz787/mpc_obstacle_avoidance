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