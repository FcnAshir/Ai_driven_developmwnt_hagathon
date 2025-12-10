---
sidebar_position: 1
---

# Lab 1: Create a ROS 2 Package

## Objective

In this lab, you will learn the basics of ROS 2 by creating your first ROS 2 package and implementing simple publisher and subscriber nodes. This foundational lab will introduce you to the ROS 2 workspace structure, package creation, and basic communication patterns.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Basic Python programming knowledge
- Understanding of ROS 2 concepts from the module readings

## Learning Outcomes

By completing this lab, you will be able to:
- Create a ROS 2 workspace and package
- Understand the structure of a ROS 2 package
- Implement basic publisher and subscriber nodes
- Build and run ROS 2 nodes
- Use ROS 2 command-line tools to monitor communication

## Tasks

1. Create a new ROS 2 workspace and package
2. Implement a publisher node that sends messages
3. Implement a subscriber node that receives messages
4. Build and run your nodes
5. Monitor communication using ROS 2 tools

## Steps

### Step 1: Create a ROS 2 Workspace

First, create a workspace directory for your ROS 2 packages:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 2: Create a New Package

Create a new ROS 2 package using the `ros2 pkg create` command:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs my_robot_interfaces ros2_package_examples
```

If the above command fails due to the custom interface dependency, create the package with basic dependencies:

```bash
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs ros2_package_examples
```

### Step 3: Create Publisher Node

In the `ros2_package_examples` package, create a publisher node. First, create the Python file:

```bash
cd ~/ros2_ws/src/ros2_package_examples
mkdir -p ros2_package_examples
```

Create the publisher node at `ros2_package_examples/talker.py`:

```python
#!/usr/bin/env python3
# talker.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create Subscriber Node

Create the subscriber node at `ros2_package_examples/listener.py`:

```python
#!/usr/bin/env python3
# listener.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Update setup.py

Update the `setup.py` file to make your nodes executable:

```python
from setuptools import find_packages, setup

package_name = 'ros2_package_examples'

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
    description='Examples for ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_package_examples.talker:main',
            'listener = ros2_package_examples.listener:main',
        ],
    },
)
```

### Step 6: Update package.xml

Update the `package.xml` file to include necessary dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2_package_examples</name>
  <version>0.0.0</version>
  <description>Examples for ROS 2 package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 7: Build and Run

1. Build your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_package_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the publisher node in one terminal:
   ```bash
   ros2 run ros2_package_examples talker
   ```

4. In another terminal, run the subscriber node:
   ```bash
   ros2 run ros2_package_examples listener
   ```

## Expected Outcome

You should see the publisher node output messages like:
```
[INFO] [1612345678.123456789] [talker]: Publishing: "Hello World: 0"
[INFO] [1612345678.623456789] [talker]: Publishing: "Hello World: 1"
```

And the subscriber node output like:
```
[INFO] [1612345678.123456789] [listener]: I heard: "Hello World: 0"
[INFO] [1612345678.623456789] [listener]: I heard: "Hello World: 1"
```

## Verification

Use ROS 2 command-line tools to verify the communication:

```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo messages on the chatter topic
ros2 topic echo /chatter std_msgs/msg/String
```

## Troubleshooting

- If you get "command not found" errors, ensure ROS 2 is properly sourced
- If nodes don't connect, check that both terminals have the workspace sourced
- If you see permission errors, make sure your Python files are executable: `chmod +x *.py`
- Ensure the package name in your code matches the actual package directory name

## Next Steps

This lab provides the foundation for more complex ROS 2 applications in subsequent labs. You now understand how to:
- Create ROS 2 packages with proper structure
- Implement publisher/subscriber communication patterns
- Use ROS 2 command-line tools for system introspection
- Build and run ROS 2 nodes