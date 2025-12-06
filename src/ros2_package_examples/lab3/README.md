# Lab 3: Robot Description and Visualization

## Overview
This lab demonstrates how to define a robot model using URDF (Unified Robot Description Format) and visualize it in RViz. You'll learn how to create robot descriptions for simulation and visualization.

## Learning Objectives
- Understand URDF robot description format
- Learn how to define robot links and joints
- Visualize robots in RViz

## Files
- `humanoid.urdf`: Robot description in URDF format
- `display.launch.py`: Launch file for visualization
- `package.xml`: Package metadata

## Theory
URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot's physical and visual properties, including links, joints, inertial properties, and visual meshes.

## Implementation
The implementation includes:
- Complete humanoid robot URDF with multiple links and joints
- Visual and collision properties for each link
- Launch file for RViz visualization

## Running the Demo
```bash
# Build the package first
colcon build --packages-select ros2_package_examples
source install/setup.bash

# Launch the visualization
ros2 launch ros2_package_examples display.launch.py
```

## Exercises
1. Add more joints to create a complete humanoid model
2. Include sensor definitions in the URDF
3. Add transmission elements for actuator control