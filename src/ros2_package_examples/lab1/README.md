# Lab 1: Basic ROS 2 Node

## Overview
This lab introduces the basic structure of a ROS 2 node in Python. You'll learn how to create a simple node that initializes and runs.

## Learning Objectives
- Understand the basic structure of a ROS 2 Python node
- Learn the node initialization process
- Understand the ROS 2 execution model

## Files
- `simple_node.py`: Basic ROS 2 node implementation
- `package.xml`: Package metadata
- `setup.py`: Python package setup

## Theory
A ROS 2 node is the fundamental building block of a ROS 2 system. Nodes contain the executable code that performs specific tasks and communicate with other nodes through topics, services, and actions.

## Implementation
The implementation includes:
- Basic node class inheriting from rclpy.Node
- Proper initialization and cleanup
- ROS 2 spin loop for message processing

## Running the Demo
```bash
# Build the package first
colcon build --packages-select ros2_package_examples
source install/setup.bash

# Run the node
ros2 run ros2_package_examples simple_node
```

## Exercises
1. Add a parameter to the node
2. Add a timer to the node that logs messages periodically
3. Create a custom message type and use it in the node