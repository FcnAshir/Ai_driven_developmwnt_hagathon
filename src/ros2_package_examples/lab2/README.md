# Lab 2: ROS 2 Publisher and Subscriber

## Overview
This lab demonstrates the publisher-subscriber communication pattern in ROS 2. You'll create two nodes that communicate using topics.

## Learning Objectives
- Understand publisher-subscriber communication
- Learn how to create publishers and subscribers
- Understand message passing in ROS 2

## Files
- `talker_node.py`: Publisher node implementation
- `listener_node.py`: Subscriber node implementation
- `package.xml`: Package metadata
- `setup.py`: Python package setup

## Theory
The publisher-subscriber pattern is a fundamental communication paradigm in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics. This enables decoupled communication between nodes.

## Implementation
The implementation includes:
- Publisher node that sends String messages
- Subscriber node that receives and logs messages
- Proper topic configuration and message handling

## Running the Demo
```bash
# Build the package first
colcon build --packages-select ros2_package_examples
source install/setup.bash

# Terminal 1: Run the publisher
ros2 run ros2_package_examples talker_node

# Terminal 2: Run the subscriber
ros2 run ros2_package_examples listener_node
```

## Exercises
1. Change the message type from String to a custom message
2. Add QoS (Quality of Service) settings to the publisher/subscriber
3. Create multiple publishers and subscribers on different topics