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