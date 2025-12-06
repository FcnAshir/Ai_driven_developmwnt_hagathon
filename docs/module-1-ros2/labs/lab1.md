# Lab 1: Create a ROS 2 Package

## Objective
Learn to create and structure a basic ROS 2 package.

## Prerequisites
- ROS 2 installed (Humble Hawksbill or later)
- Basic understanding of Linux command line
- Python 3.8 or later

## Steps

### 1. Set up the workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Create a new ROS 2 package
```bash
ros2 pkg create --build-type ament_python my_robot_package
```

### 3. Explore the package structure
```bash
cd my_robot_package
ls -la
```

The package structure includes:
- `package.xml`: Package metadata and dependencies
- `setup.py`: Python package configuration
- `setup.cfg`: Installation configuration
- `my_robot_package/`: Python module directory
- `test/`: Test directory

### 4. Create a simple Python node
Create a new file `my_robot_package/simple_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('SimpleNode has been started')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5. Update setup.py to include the executable
Edit `setup.py` to add the entry point:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple ROS 2 package for learning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = my_robot_package.simple_node:main',
        ],
    },
)
```

### 6. Build and run the package
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
ros2 run my_robot_package simple_node
```

## Expected Output
You should see a message indicating that the SimpleNode has been started.

## Troubleshooting
- If you get a "command not found" error, ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
- If the package fails to build, check for syntax errors in your Python files
- Make sure your Python files have proper indentation and syntax

## Next Steps
- Explore different node structures and communication patterns
- Learn about topics, services, and actions
- Create more complex ROS 2 packages with multiple nodes