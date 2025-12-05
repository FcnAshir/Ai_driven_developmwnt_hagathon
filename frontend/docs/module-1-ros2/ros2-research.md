# ROS 2 Research Summary

## Primary Sources

### Official ROS 2 Documentation
- **ROS 2 Documentation**: https://docs.ros.org/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **ROS 2 Concepts**: https://docs.ros.org/en/humble/Concepts.html
- **ROS 2 Design**: https://design.ros2.org/

### ROS 2 Distributions
- **Humble Hawksbill** (LTS): Current long-term support version
- **Iron Irwini**: Current stable version
- **Rolling Ridley**: Development version

### Core ROS 2 Concepts
- **Nodes**: Basic computational elements
- **Topics**: Communication channels for asynchronous messaging
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous request/response with feedback and goal preemption
- **Parameters**: Configuration values accessible at runtime
- **Launch files**: XML/YAML files for starting multiple nodes
- **Packages**: Organizational units containing code, data, and configuration

### DDS (Data Distribution Service) Integration
- ROS 2 uses DDS as the middleware for communication
- Multiple DDS implementations supported (Fast DDS, Cyclone DDS, RTI Connext DDS)

### Build System
- **colcon**: Multi-package build system
- **ament**: ROS 2's build system and package format

### ROS 2 Tools
- **ros2 run**: Run a node
- **ros2 topic**: Interact with topics
- **ros2 service**: Interact with services
- **ros2 action**: Interact with actions
- **ros2 param**: Interact with parameters
- **rqt**: GUI tools for introspection
- **rviz2**: 3D visualization tool

### URDF (Unified Robot Description Format)
- XML format for representing robot models
- Used for kinematics, dynamics, and visualization

### ROS 2 for Humanoid Robotics
- **ros2_control**: Hardware abstraction and controller management
- **MoveIt 2**: Motion planning framework
- **navigation2**: Navigation stack for mobile robots
- **robot_state_publisher**: Publishes joint states to TF