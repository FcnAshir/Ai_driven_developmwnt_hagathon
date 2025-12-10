---
sidebar_position: 2
---

# Lab 2: Publish and Subscribe to Topics

## Objective

In this lab, you will deepen your understanding of ROS 2 topics by implementing more complex publisher and subscriber nodes. You'll work with different message types, Quality of Service (QoS) settings, and explore advanced communication patterns that are essential for humanoid robotics applications.

## Prerequisites

- Completion of Lab 1
- Understanding of basic ROS 2 concepts (nodes, topics, messages)
- Basic Python programming skills

## Learning Outcomes

By completing this lab, you will be able to:
- Implement publishers and subscribers for different message types
- Configure Quality of Service (QoS) profiles for specific use cases
- Use custom message types in ROS 2
- Monitor and debug topic communication
- Apply best practices for topic-based communication

## Tasks

1. Implement a publisher for sensor data (Image, LaserScan, or IMU)
2. Implement a subscriber that processes the sensor data
3. Configure appropriate QoS settings for real-time performance
4. Create and use a custom message type
5. Monitor communication performance and reliability

## Steps

### Step 1: Choose a Sensor Message Type

Select one of the following sensor message types for your implementation:
- `sensor_msgs/msg/Image` - For camera data
- `sensor_msgs/msg/LaserScan` - For LiDAR data
- `sensor_msgs/msg/Imu` - For inertial measurement unit data

For this lab, we'll use `sensor_msgs/msg/LaserScan` as an example.

### Step 2: Create the Publisher Node

Create a publisher node that simulates sensor data. Create the file `src/ros2_package_examples/ros2_package_examples/laser_publisher.py`:

```python
#!/usr/bin/env python3
# laser_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import random


class LaserPublisherNode(Node):

    def __init__(self):
        super().__init__('laser_publisher')

        # Create publisher with specific QoS settings for sensor data
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(LaserScan, 'scan', qos_profile)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.scan_count = 0

    def timer_callback(self):
        msg = LaserScan()

        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set laser scan parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Calculate number of ranges
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = []

        # Generate simulated range data with some obstacles
        for i in range(num_ranges):
            # Simulate a range reading with some random noise
            base_distance = 2.0 + 1.5 * math.sin(i * msg.angle_increment * 2)
            noise = random.uniform(-0.1, 0.1)
            range_val = max(msg.range_min, min(msg.range_max, base_distance + noise))
            msg.ranges.append(range_val)

        # Set intensities (optional)
        msg.intensities = [100.0] * num_ranges

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing laser scan {self.scan_count}')
        self.scan_count += 1


def main(args=None):
    rclpy.init(args=args)
    laser_publisher = LaserPublisherNode()
    rclpy.spin(laser_publisher)
    laser_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create the Subscriber Node

Create a subscriber node that processes the laser scan data. Create the file `src/ros2_package_examples/ros2_package_examples/laser_subscriber.py`:

```python
#!/usr/bin/env python3
# laser_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LaserSubscriberNode(Node):

    def __init__(self):
        super().__init__('laser_subscriber')

        # Create subscriber with matching QoS settings
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Process the laser scan data
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} ranges')

        # Find minimum distance (closest obstacle)
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_distance = min(valid_ranges)
                min_idx = msg.ranges.index(min_distance)
                angle_to_obstacle = msg.angle_min + min_idx * msg.angle_increment

                self.get_logger().info(
                    f'Closest obstacle: {min_distance:.2f}m at angle {math.degrees(angle_to_obstacle):.1f}°'
                )

                # Check if there's an obstacle in front (within 30 degrees of center)
                center_idx = len(msg.ranges) // 2
                front_range = msg.ranges[center_idx]

                if front_range < 1.0:  # Obstacle within 1 meter in front
                    self.get_logger().warn('OBSTACLE DETECTED IN FRONT!')
                else:
                    self.get_logger().info('Path ahead is clear')
            else:
                self.get_logger().info('No valid range measurements')


def main(args=None):
    rclpy.init(args=args)
    laser_subscriber = LaserSubscriberNode()
    rclpy.spin(laser_subscriber)
    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create a Custom Message (Optional)

For more complex applications, you might want to create a custom message. Create a directory for custom messages:

```bash
mkdir -p src/ros2_package_examples/ros2_package_examples/msg
```

Create a custom message definition at `src/ros2_package_examples/ros2_package_examples/msg/Obstacle.msg`:

```
# Obstacle.msg
# Represents an obstacle detected by the robot

float32 distance
float32 angle
float32 size
string type  # "static", "dynamic", "unknown"
```

### Step 5: Update setup.py

Add the new nodes to the `setup.py` file:

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
            'laser_publisher = ros2_package_examples.laser_publisher:main',
            'laser_subscriber = ros2_package_examples.laser_subscriber:main',
        ],
    },
)
```

### Step 6: Update package.xml

Update the `package.xml` to include sensor message dependencies:

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
  <depend>sensor_msgs</depend>

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
   ros2 run ros2_package_examples laser_publisher
   ```

4. In another terminal, run the subscriber node:
   ```bash
   ros2 run ros2_package_examples laser_subscriber
   ```

## Expected Outcome

The publisher should output laser scan data at 10Hz, and the subscriber should process this data to detect obstacles. You should see output like:

Publisher:
```
[INFO] [1612345678.123456789] [laser_publisher]: Publishing laser scan 0
[INFO] [1612345678.223456789] [laser_publisher]: Publishing laser scan 1
```

Subscriber:
```
[INFO] [1612345678.123456789] [laser_subscriber]: Received laser scan with 181 ranges
[INFO] [1612345678.123456789] [laser_subscriber]: Closest obstacle: 1.23m at angle -15.0°
[INFO] [1612345678.123456789] [laser_subscriber]: Path ahead is clear
```

## Quality of Service (QoS) Considerations

For humanoid robotics applications, QoS settings are crucial:
- **Sensor Data**: Use BEST_EFFORT reliability with small history depth for real-time performance
- **Control Commands**: Use RELIABLE reliability with strict deadlines
- **Map Data**: Use TRANSIENT_LOCAL durability to ensure new nodes receive current maps

## Verification

Use ROS 2 tools to verify communication:

```bash
# Monitor the scan topic
ros2 topic echo /scan sensor_msgs/msg/LaserScan

# Check the topic type
ros2 topic info /scan

# Monitor bandwidth usage
ros2 topic hz /scan
```

## Troubleshooting

- If you get import errors, ensure sensor_msgs is installed: `sudo apt install ros-humble-sensor-msgs`
- Check that QoS profiles match between publisher and subscriber
- Verify that frame IDs are properly set in the header
- Use `rqt_plot` to visualize numerical data from topics

## Next Steps

This lab demonstrates advanced topic usage with sensor data, which is fundamental for humanoid robotics perception systems. You now understand how to:
- Work with complex message types like sensor data
- Configure appropriate QoS settings for different data types
- Process and analyze sensor information in real-time
- Implement best practices for sensor data handling