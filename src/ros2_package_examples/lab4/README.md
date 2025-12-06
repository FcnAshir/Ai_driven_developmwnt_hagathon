# Lab 4: Joint Control and State Publishing

## Overview
This lab demonstrates joint control for robotic systems by publishing joint state messages. You'll learn how to control robot joints and publish their states for visualization and control systems.

## Learning Objectives
- Understand joint state messages in ROS 2
- Learn how to publish joint positions, velocities, and efforts
- Implement joint control algorithms
- Coordinate joint movements for robot behavior

## Files
- `joint_controller.py`: Basic joint controller implementation
- `advanced_joint_controller.py`: Advanced joint control with trajectories
- `joint_control.launch.py`: Launch file for joint control nodes
- `package.xml`: Package metadata
- `setup.py`: Python package setup

## Theory
Joint state messages are crucial for robot control and visualization. They provide real-time information about joint positions, velocities, and efforts, enabling robot state estimation and control algorithms.

## Implementation
The implementation includes:
- Joint state publisher with oscillating motion patterns
- Different control patterns for different joint types
- Proper timestamping and frame ID for joint states
- Launch file for easy execution

## Running the Demo
```bash
# Build the package first
colcon build --packages-select ros2_package_examples
source install/setup.bash

# Launch the joint control system
ros2 launch ros2_package_examples joint_control.launch.py
```

## Exercises
1. Implement inverse kinematics for end-effector control
2. Add joint limits and safety constraints
3. Create trajectory following capabilities for smooth motion