# ROS2 Package Examples

This package provides ROS 2 examples for humanoid robotics applications, demonstrating fundamental concepts from basic nodes to advanced action servers.

## Overview

The ROS2 Package Examples include 5 labs that progressively build ROS 2 capabilities:

- **Lab 1**: Basic ROS 2 node structure and execution
- **Lab 2**: Publisher-subscriber communication pattern
- **Lab 3**: Robot description and visualization with URDF
- **Lab 4**: Joint control and state publishing
- **Lab 5**: Actions and LLM integration for complex tasks

## Components

### Basic ROS 2 Concepts
- Node creation and lifecycle management
- Publisher and subscriber implementation
- Message passing and communication

### Robot Description
- URDF robot modeling
- RViz visualization
- Robot state publishing

### Advanced Communication
- Action servers for long-running tasks
- Feedback and progress reporting
- Integration with external systems (LLMs)

## Usage

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_package_examples
source install/setup.bash
```

### 2. Run Individual Components
```bash
# Lab 1: Basic node
ros2 run ros2_package_examples simple_node

# Lab 2: Publisher and subscriber
ros2 run ros2_package_examples talker_node
ros2 run ros2_package_examples listener_node

# Lab 3: Robot visualization
ros2 launch ros2_package_examples display.launch.py

# Lab 4: Joint control
ros2 run ros2_package_examples joint_controller

# Lab 5: Action server
ros2 run ros2_package_examples robot_executor_server
ros2 run ros2_package_examples llm_client
```

## Files Structure

```
ros2_package_examples/
├── ros2_package_examples/
│   ├── __init__.py
│   ├── simple_node.py
│   ├── talker_node.py
│   ├── listener_node.py
│   ├── joint_controller.py
│   ├── advanced_joint_controller.py
│   ├── robot_executor_server.py
│   ├── llm_client.py
│   └── llm_interface.py
├── lab1/
├── lab2/
├── lab3/
├── lab4/
├── lab5/
├── urdf/
├── launch/
├── action/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── README.md
```

## Dependencies

- ROS 2 (Humble Hawksbill or later)
- rclpy (Python ROS client library)
- Standard ROS message packages (std_msgs, sensor_msgs, etc.)
- RViz2 for visualization

## Troubleshooting

- Ensure ROS 2 environment is properly sourced
- Check that all dependencies are installed
- For action servers, ensure the action definition is built first
- Verify proper network configuration for multi-node communication