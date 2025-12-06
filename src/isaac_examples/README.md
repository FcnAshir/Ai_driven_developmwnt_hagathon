# Isaac Examples Package

This package provides Isaac Sim examples for humanoid robotics simulation, demonstrating various capabilities from basic setup to advanced scenarios.

## Overview

The Isaac Examples package includes 6 labs that progressively build complex simulation capabilities:

- **Lab 1**: Basic Isaac Sim setup and environment creation
- **Lab 2**: Robot control and articulation
- **Lab 3**: Sensor simulation (cameras, LiDAR, IMU)
- **Lab 4**: Terrain generation and environment design
- **Lab 5**: AI integration with perception and control
- **Lab 6**: Advanced scenarios with multi-robot coordination

## Components

### Simulation Environment
- Basic scene setup with physics
- Environment generation tools
- Multi-robot support

### Robot Control
- Joint position and velocity control
- Cartesian movement
- Kinematic control

### Sensor Simulation
- Camera simulation with realistic parameters
- LiDAR point cloud generation
- IMU data simulation

### AI Integration
- Perception pipeline integration
- Control algorithm connection
- Learning pipeline support

## Usage

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select isaac_examples
source install/setup.bash
```

### 2. Run Individual Labs
```bash
# Lab 1: Basic setup
python3 src/isaac_examples/isaac_examples/isaac_basic_setup.py

# Lab 2: Robot control
python3 src/isaac_examples/isaac_examples/isaac_robot_control.py

# Lab 3: Sensor simulation
python3 src/isaac_examples/isaac_examples/isaac_sensor_simulation.py

# Lab 4: Terrain generation
python3 src/isaac_examples/isaac_examples/isaac_terrain_generation.py

# Lab 5: AI integration
python3 src/isaac_examples/isaac_examples/isaac_ai_integration.py

# Lab 6: Advanced scenarios
python3 src/isaac_examples/isaac_examples/isaac_advanced_scenarios.py
```

### 3. Or Use ROS 2 Launch
```bash
ros2 run isaac_examples isaac_basic_setup
ros2 run isaac_examples isaac_robot_control
```

## Files Structure

```
isaac_examples/
├── isaac_examples/
│   ├── __init__.py
│   ├── isaac_basic_setup.py
│   ├── isaac_robot_control.py
│   ├── isaac_sensor_simulation.py
│   ├── isaac_terrain_generation.py
│   ├── isaac_ai_integration.py
│   └── isaac_advanced_scenarios.py
├── lab1/
├── lab2/
├── lab3/
├── lab4/
├── lab5/
├── lab6/
├── launch/
├── test/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── README.md
```

## Dependencies

- NVIDIA Isaac Sim (for full functionality)
- ROS 2 (for robot communication)
- NumPy, PyTorch, and other standard Python libraries

## Troubleshooting

- Ensure Isaac Sim is properly installed and licensed
- Check that ROS 2 environment is sourced
- Verify Python dependencies are installed
- For full simulation features, run within Isaac Sim environment