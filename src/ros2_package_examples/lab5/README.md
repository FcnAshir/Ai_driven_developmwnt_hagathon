# Lab 5: ROS 2 Actions and LLM Integration

## Overview
This lab demonstrates ROS 2 actions for long-running tasks and integration with Large Language Models (LLMs). You'll learn how to create action servers and clients for complex robot tasks.

## Learning Objectives
- Understand ROS 2 actions for long-running tasks
- Create action servers and clients
- Integrate with external systems (LLMs)
- Handle feedback and progress reporting

## Files
- `RobotCommand.action`: Action definition file
- `robot_executor_server.py`: Action server implementation
- `llm_interface.py`: Interface to LLM systems
- `llm_client.py`: Action client for LLM commands
- `package.xml`: Package metadata
- `setup.py`: Python package setup

## Theory
ROS 2 actions are used for long-running tasks that require feedback and the ability to cancel. They consist of goals, results, and feedback, making them ideal for complex robot behaviors and task execution.

## Implementation
The implementation includes:
- Custom action definition for robot commands
- Action server that executes robot tasks
- LLM interface for natural language processing
- Action client for sending commands

## Running the Demo
```bash
# Build the package first (this will generate action files)
colcon build --packages-select ros2_package_examples
source install/setup.bash

# Terminal 1: Run the action server
ros2 run ros2_package_examples robot_executor_server

# Terminal 2: Run the LLM client
ros2 run ros2_package_examples llm_client
```

## Exercises
1. Add more complex action types with multiple stages
2. Implement action preemption and cancellation
3. Create a more sophisticated LLM interface with context management