# Gazebo and Unity Research Summary

## Primary Sources

### Gazebo Documentation
- **Gazebo Classic**: http://gazebosim.org/
- **Ignition Gazebo**: https://ignitionrobotics.org/
- **Gazebo Garden**: Latest version of Ignition Gazebo
- **Gazebo Fortress**: LTS version

### Unity Documentation
- **Unity ML-Agents**: https://github.com/Unity-Technologies/ml-agents
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **Unity ROS TCP Connector**: Bridge between Unity and ROS/ROS 2

### Gazebo Core Concepts
- **SDF (Simulation Description Format)**: XML format for describing simulation environments
- **Gazebo Plugins**: C++ plugins for custom simulation behavior
- **Physics Engines**: Support for ODE, Bullet, DART, and Simbody
- **Sensors**: Support for cameras, LiDAR, IMU, force/torque, etc.
- **Models**: 3D robot models and environment objects

### Unity for Robotics
- **ROS TCP Connector**: Enables communication between Unity and ROS/ROS 2
- **ML-Agents**: Reinforcement learning framework
- **Physics**: Built-in physics engine for simulation
- **High-fidelity rendering**: Realistic visual simulation
- **XR Support**: Virtual and augmented reality capabilities

### Integration Approaches
- **Gazebo ↔ ROS 2**: Native integration, physics-focused simulation
- **Unity ↔ ROS 2**: High-fidelity visualization, ML training
- **Combined Pipeline**: Use Gazebo for physics, Unity for visualization

### Digital Twin Concepts
- **Simulation Fidelity**: Balance between accuracy and performance
- **Sensor Simulation**: Accurate modeling of real-world sensors
- **Sim-to-Real Transfer**: Techniques to bridge simulation and real-world performance
- **Hardware-in-the-loop**: Integration with real hardware during simulation

### Gazebo for Humanoid Robotics
- **Humanoid Model Support**: URDF/SDF support for humanoid robots
- **Physics Simulation**: Accurate joint dynamics and contact physics
- **Terrain Generation**: Support for complex environments
- **Multi-robot Simulation**: Simultaneous simulation of multiple robots