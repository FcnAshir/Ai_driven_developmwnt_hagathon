# Lab 4: Control a Joint using rclpy

## Objective
Programmatically control robot joints using the ROS 2 Python client library (`rclpy`).

## Prerequisites
- Completed Lab 3 (URDF model)
- ROS 2 with Joint State Publisher
- Basic understanding of robot kinematics
- Python programming knowledge

## Steps

### 1. Create a joint controller package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python joint_controller_pkg
cd joint_controller_pkg
```

### 2. Create the joint controller node
Create `joint_controller_pkg/joint_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Initialize joint positions for a simple oscillating motion
        self.time_step = 0.0

        # Define joint names based on our humanoid URDF
        self.joint_names = [
            'neck_joint', 'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'left_hip_joint',
            'left_knee_joint', 'right_hip_joint', 'right_knee_joint'
        ]

        self.get_logger().info('Joint Controller Node Started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = []

        # Update time step
        self.time_step += 0.05

        # Generate oscillating joint positions
        for i, joint_name in enumerate(self.joint_names):
            # Different oscillation patterns for different joints
            if 'neck' in joint_name:
                # Gentle head movement
                position = 0.2 * math.sin(self.time_step)
            elif 'shoulder' in joint_name:
                # Shoulder movement
                position = 0.5 * math.sin(self.time_step + i)
            elif 'elbow' in joint_name:
                # Elbow movement
                position = 0.3 * math.sin(self.time_step * 1.5 + i)
            elif 'hip' in joint_name:
                # Hip movement
                position = 0.4 * math.sin(self.time_step * 0.8 + i)
            elif 'knee' in joint_name:
                # Knee movement
                position = 0.4 * math.sin(self.time_step * 0.8 + i + 1.5)
            else:
                # Default position
                position = 0.0

            msg.position.append(position)

        # Add timestamp and frame
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish the message
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Create a more advanced joint controller with target positions
Create `joint_controller_pkg/advanced_joint_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class AdvancedJointController(Node):
    def __init__(self):
        super().__init__('advanced_joint_controller')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Initialize joint positions and velocities
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0
        }

        # Target positions for movement
        self.target_positions = self.joint_positions.copy()

        # Movement parameters
        self.move_speed = 0.02  # radians per update
        self.time_step = 0.0

        # Create a simple walking gait pattern
        self.gait_phase = 0.0
        self.gait_period = 4.0  # seconds for one gait cycle

        self.get_logger().info('Advanced Joint Controller Node Started')

    def publish_joint_states(self):
        # Update joint positions toward targets
        self.update_joint_positions()

        # Create and publish joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [0.0] * len(msg.position)  # Simplified
        msg.effort = [0.0] * len(msg.position)    # Simplified

        # Add timestamp and frame
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish the message
        self.joint_pub.publish(msg)

        # Update gait phase
        self.gait_phase += 0.05
        if self.gait_phase >= self.gait_period:
            self.gait_phase = 0.0

    def update_joint_positions(self):
        # Implement a simple walking gait
        left_leg_phase = math.sin(2 * math.pi * self.gait_phase / self.gait_period)
        right_leg_phase = math.sin(2 * math.pi * self.gait_phase / self.gait_period + math.pi)

        # Update leg joints for walking motion
        self.joint_positions['left_hip_joint'] = 0.2 * left_leg_phase
        self.joint_positions['left_knee_joint'] = 0.3 * abs(left_leg_phase)
        self.joint_positions['right_hip_joint'] = 0.2 * right_leg_phase
        self.joint_positions['right_knee_joint'] = 0.3 * abs(right_leg_phase)

        # Add some arm movement for balance
        self.joint_positions['left_shoulder_joint'] = 0.1 * right_leg_phase
        self.joint_positions['right_shoulder_joint'] = 0.1 * left_leg_phase


def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Update setup.py to include executables
Edit `setup.py`:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'joint_controller_pkg'

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
    description='Joint controller for humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = joint_controller_pkg.joint_controller:main',
            'advanced_joint_controller = joint_controller_pkg.advanced_joint_controller:main',
        ],
    },
)
```

### 5. Create a launch file to run the controller with visualization
Create `launch/joint_control.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_humanoid_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read()
        }]
    )

    joint_controller = Node(
        package='joint_controller_pkg',
        executable='advanced_joint_controller',
        name='advanced_joint_controller',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_controller,
        rviz2
    ])
```

### 6. Build and run the joint controller
```bash
cd ~/ros2_ws
colcon build --packages-select joint_controller_pkg
source install/setup.bash
ros2 launch joint_controller_pkg joint_control.launch.py
```

## Expected Output
RViz2 should display the humanoid robot model with joints moving in a coordinated walking pattern. The joint controller should continuously publish joint state messages that update the robot's pose in the visualization.

## Troubleshooting
- If joints don't move, check that the joint names in the controller match those in the URDF
- Verify that the joint state publisher is receiving messages by using `ros2 topic echo /joint_states`
- Check that the robot model loads correctly in RViz2

## Next Steps
- Implement inverse kinematics for more complex movements
- Add PID controllers for precise joint control
- Integrate with Gazebo simulation for physics-based control