# Path Smoothing and Trajectory Control in 2D Space

ROS2 Humble | Python | Ubuntu 22.04 | Differential Drive Robot

## What This Does

This package implements all 3 assignment tasks:

1. Path Smoothing - Cubic spline interpolation through discrete waypoints
2. Trajectory Generation - Time-stamped trajectory with trapezoidal velocity profile
3. Trajectory Tracking - Proportional controller with cross-track error logging

## Prerequisites

sudo apt install -y ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
pip3 install numpy scipy pytest

## Build

cd ~/ros2_ws
colcon build --packages-select path_control
source install/setup.bash

## Run

Terminal 1:
ros2 launch path_control path_control.launch.py

Terminal 2:
rviz2 -d ~/ros2_ws/src/path_control/rviz/path_control.rviz

## Run Tests

cd ~/ros2_ws/src/path_control
python3 -m pytest test/test_path_control.py -v

15 unit tests covering path continuity, timestamp monotonicity, unicycle model physics, and cross-track error geometry.

## What You See in RViz

Red spheres = original discrete waypoints
Blue curved line = smooth cubic spline path
Blue arrows = trajectory direction vectors
Green arrow = robot moving in real time
Yellow line = robot actual traveled trace

## Algorithm Design

### 1. Path Smoothing - Cubic Spline

Waypoints are parameterized by cumulative arc length. Two independent cubic splines fit x(t) and y(t). This gives C2 continuity meaning smooth curvature at every point. 200 points sampled uniformly. Path passes exactly through all waypoints.

Why cubic spline over Bezier: cubic spline guarantees passage through all waypoints. Bezier only approximates them.

### 2. Trajectory Generation - Trapezoidal Velocity Profile

Phase 1 first 20 percent: accelerate from 0 to 0.5 m/s
Phase 2 middle 60 percent: cruise at 0.5 m/s
Phase 3 last 20 percent: decelerate back to 0

Timestamps assigned by integrating dt = ds divided by v(s) along arc length.

### 3. Controller - Proportional

At each timestep at 50 Hz the controller selects the current target point, computes angle error between heading and direction to target, sets linear velocity 0.15 m/s constant, sets angular velocity 0.8 times angle error. Advances to next waypoint when within 0.15m. Cross-track error logged every 20 steps, consistently under 0.01m.

## Node Architecture

path_smoother publishes /smooth_path
trajectory_generator subscribes to /smooth_path and publishes /trajectory_viz
trajectory_controller subscribes to /smooth_path and publishes /cmd_vel and /robot_marker and /robot_trace
All markers go to RViz2

## Extending to a Real Robot

1. Remove simulated unicycle integration in trajectory_controller.py
2. Subscribe to /odom for real robot pose
3. /cmd_vel already publishes, connect it to your robot driver
4. Add AMCL for localization
5. Tune linear_vel and the 0.8 gain for your hardware

## Obstacle Avoidance Extension

Subscribe to /scan from LiDAR. For each point within safety radius compute repulsive force vector away from obstacle. Blend with controller output: angular_vel_final = angular_vel_controller + angular_vel_repulsive. Alternatively insert virtual waypoints around obstacles and re-run spline smoother.

## AI Tools Used

Claude (Anthropic) assisted with ROS2 boilerplate and README formatting. All core algorithms including cubic spline parameterization, trapezoidal velocity integration, proportional controller, cross-track error formula, and unit tests were implemented and verified by the developer.

## File Structure

launch/path_control.launch.py - launches all 3 nodes
path_control/path_smoother.py - Task 1 cubic spline smoothing
path_control/trajectory_generator.py - Task 2 time-stamped trajectory
path_control/trajectory_controller.py - Task 3 proportional controller
rviz/path_control.rviz - pre-configured RViz display
test/test_path_control.py - 15 unit tests
