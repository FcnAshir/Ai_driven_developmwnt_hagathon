---
sidebar_position: 4
---

# Lab 4: Control a Joint using rclpy

## Objective

In this lab, you will learn how to implement joint control for a robot using ROS 2 and the rclpy client library. You'll create a joint controller node that can receive commands and control a specific joint, learning about real-time control considerations and feedback mechanisms essential for humanoid robotics.

## Prerequisites

- Understanding of ROS 2 concepts (topics, services, messages)
- Basic knowledge of robot kinematics and joint control
- Completion of previous labs, especially the URDF lab

## Learning Outcomes

By completing this lab, you will be able to:
- Implement a joint controller using ROS 2 topics and services
- Understand the difference between position, velocity, and effort control
- Work with JointState and JointTrajectory messages
- Implement basic control algorithms (PID) in ROS 2
- Handle real-time control requirements and timing constraints

## Tasks

1. Create a joint controller node that can receive position commands
2. Implement a simple PID controller for joint position control
3. Subscribe to joint state feedback and publish control commands
4. Create a service to set joint parameters (gains, limits)
5. Test the controller with the URDF robot model from Lab 3

## Steps

### Step 1: Understand Joint Control Concepts

In robotics, joint control typically involves:
- **Position Control**: Command a specific joint angle/position
- **Velocity Control**: Command a specific joint velocity
- **Effort/Torque Control**: Command a specific joint torque
- **Trajectory Control**: Command a sequence of positions over time

### Step 2: Create the Joint Controller Node

Create a joint controller that implements position control. Create the file `src/ros2_package_examples/ros2_package_examples/joint_controller.py`:

```python
#!/usr/bin/env python3
# joint_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')

        # Joint name to control (for this example, we'll control the left elbow)
        self.joint_name = 'left_elbow'

        # Control parameters
        self.kp = 5.0   # Proportional gain
        self.ki = 0.1   # Integral gain
        self.kd = 0.5   # Derivative gain

        # Control variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_effort = 0.0
        self.setpoint = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.joint_name}_controller/joint_trajectory',
            10
        )

        # Timer for control loop (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_callback)

        # Timer for publishing commands (50Hz)
        self.command_timer = self.create_timer(0.02, self.publish_command)

        self.get_logger().info(f'Joint controller initialized for {self.joint_name}')

    def joint_state_callback(self, msg):
        """Callback to receive joint state feedback"""
        try:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            if len(msg.velocity) > idx:
                self.current_velocity = msg.velocity[idx]
            if len(msg.effort) > idx:
                self.current_effort = msg.effort[idx]
        except ValueError:
            # Joint name not found in message
            pass

    def control_callback(self):
        """Main control loop - compute control output using PID"""
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9

        if dt > 0:
            # Calculate error
            error = self.setpoint - self.current_position

            # Proportional term
            p_term = self.kp * error

            # Integral term (with anti-windup)
            self.integral += error * dt
            # Limit integral to prevent windup
            self.integral = max(min(self.integral, 10.0), -10.0)
            i_term = self.ki * self.integral

            # Derivative term
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            d_term = self.kd * derivative

            # Calculate control output
            output = p_term + i_term + d_term

            # Apply limits to control output
            output = max(min(output, 10.0), -10.0)

            # Store values for next iteration
            self.previous_error = error
            self.previous_time = current_time

            # Update setpoint based on some trajectory or input
            # For this example, we'll create a simple oscillating setpoint
            current_time_sec = current_time.nanoseconds / 1e9
            self.setpoint = 0.5 + 0.3 * math.sin(current_time_sec * 0.5)

    def publish_command(self):
        """Publish joint trajectory command"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.setpoint]
        point.velocities = [0.0]  # We'll calculate this based on the trajectory
        point.accelerations = [0.0]
        point.time_from_start = Duration(sec=0, nanosec=20000000)  # 20ms

        traj_msg.points = [point]

        self.command_pub.publish(traj_msg)

    def set_gains(self, kp, ki, kd):
        """Set PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.get_logger().info(f'PID gains updated: Kp={kp}, Ki={ki}, Kd={kd}')

    def set_setpoint(self, position):
        """Set the target position"""
        self.setpoint = position
        self.get_logger().info(f'Setpoint updated to: {position}')


def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()

    try:
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        pass

    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create a Joint Trajectory Controller Node

Create a more sophisticated controller that can handle trajectory commands. Create the file `src/ros2_package_examples/ros2_package_examples/trajectory_controller.py`:

```python
#!/usr/bin/env python3
# trajectory_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
import math
from collections import deque


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        # Controller parameters
        self.joint_name = 'left_elbow'
        self.kp = 10.0
        self.ki = 1.0
        self.kd = 0.1

        # Current state
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_effort = 0.0

        # Controller state
        self.setpoint_position = 0.0
        self.setpoint_velocity = 0.0
        self.setpoint_acceleration = 0.0

        # PID state
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        # Trajectory queue
        self.trajectory_queue = deque()
        self.active_trajectory = None
        self.trajectory_start_time = None

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            f'/{self.joint_name}_trajectory',
            self.trajectory_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.joint_name}_command',
            10
        )

        self.state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            f'/{self.joint_name}_state',
            10
        )

        # Timer for control loop (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info(f'Trajectory controller initialized for {self.joint_name}')

    def joint_state_callback(self, msg):
        """Receive joint state feedback"""
        try:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            if len(msg.velocity) > idx:
                self.current_velocity = msg.velocity[idx]
            if len(msg.effort) > idx:
                self.current_effort = msg.effort[idx]
        except ValueError:
            pass

    def trajectory_callback(self, msg):
        """Receive trajectory commands"""
        # Check if this trajectory is for our joint
        if self.joint_name in msg.joint_names:
            self.trajectory_queue.clear()  # Clear existing trajectory
            self.trajectory_queue.extend(msg.points)
            self.trajectory_start_time = self.get_clock().now()
            self.get_logger().info(f'Received trajectory with {len(msg.points)} points')

    def control_loop(self):
        """Main control loop"""
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9

        # Process trajectory if available
        if self.trajectory_queue and not self.active_trajectory:
            # Start next trajectory point
            self.active_trajectory = self.trajectory_queue.popleft()
            self.trajectory_start_time = current_time

        if self.active_trajectory and self.trajectory_start_time:
            # Calculate progress along trajectory
            elapsed = (current_time - self.trajectory_start_time).nanoseconds / 1e9
            total_time = self.active_trajectory.time_from_start.sec + self.active_trajectory.time_from_start.nanosec / 1e9

            if elapsed >= total_time:
                # Move to next point or finish trajectory
                self.setpoint_position = self.active_trajectory.positions[0]
                if len(self.active_trajectory.velocities) > 0:
                    self.setpoint_velocity = self.active_trajectory.velocities[0]
                if len(self.active_trajectory.accelerations) > 0:
                    self.setpoint_acceleration = self.active_trajectory.accelerations[0]

                # Clear active trajectory and get next one if available
                self.active_trajectory = None
            else:
                # Interpolate to current time
                progress = elapsed / total_time
                if progress > 1.0:
                    progress = 1.0

                # Simple linear interpolation
                if len(self.active_trajectory.positions) > 0:
                    self.setpoint_position = self.active_trajectory.positions[0] * progress
                if len(self.active_trajectory.velocities) > 0:
                    self.setpoint_velocity = self.active_trajectory.velocities[0] * progress
                if len(self.active_trajectory.accelerations) > 0:
                    self.setpoint_acceleration = self.active_trajectory.accelerations[0] * progress

        # PID control
        if dt > 0:
            error = self.setpoint_position - self.current_position

            # Proportional term
            p_term = self.kp * error

            # Integral term
            self.integral_error += error * dt
            # Anti-windup
            self.integral_error = max(min(self.integral_error, 10.0), -10.0)
            i_term = self.ki * self.integral_error

            # Derivative term
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            d_term = self.kd * derivative

            # Calculate control output
            control_output = p_term + i_term + d_term

            # Update previous values
            self.previous_error = error
            self.previous_time = current_time

            # Publish controller state
            state_msg = JointTrajectoryControllerState()
            state_msg.joint_names = [self.joint_name]
            state_msg.desired.positions = [self.setpoint_position]
            state_msg.desired.velocities = [self.setpoint_velocity]
            state_msg.actual.positions = [self.current_position]
            state_msg.actual.velocities = [self.current_velocity]
            state_msg.error.positions = [error]
            self.state_pub.publish(state_msg)

            # Publish command (simplified - in real systems this would go to hardware interface)
            cmd_msg = JointTrajectory()
            cmd_msg.joint_names = [self.joint_name]
            point = JointTrajectoryPoint()
            point.positions = [self.setpoint_position]
            point.velocities = [self.setpoint_velocity]
            point.effort = [control_output]  # Simplified effort command
            point.time_from_start = Duration(sec=0, nanosec=10000000)  # 10ms
            cmd_msg.points = [point]
            self.command_pub.publish(cmd_msg)

    def set_gains(self, kp, ki, kd):
        """Set PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd


def main(args=None):
    rclpy.init(args=args)
    trajectory_controller = TrajectoryController()

    try:
        rclpy.spin(trajectory_controller)
    except KeyboardInterrupt:
        pass

    trajectory_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create a Joint Command Publisher for Testing

Create a simple node to publish joint commands for testing. Create the file `src/ros2_package_examples/ros2_package_examples/joint_commander.py`:

```python
#!/usr/bin/env python3
# joint_commander.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class JointCommander(Node):

    def __init__(self):
        super().__init__('joint_commander')

        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/left_elbow_trajectory',
            10
        )

        # Timer to send commands (1Hz)
        self.command_timer = self.create_timer(1.0, self.send_trajectory_command)

        self.command_count = 0
        self.get_logger().info('Joint commander initialized')

    def send_trajectory_command(self):
        """Send a trajectory command to the joint"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['left_elbow']

        # Create a trajectory point
        point = JointTrajectoryPoint()

        # Create a sinusoidal trajectory
        amplitude = 0.5
        frequency = 0.5
        phase = self.command_count * 0.5  # Increment phase each time

        target_position = amplitude * math.sin(phase)
        target_velocity = amplitude * frequency * math.cos(phase)

        point.positions = [target_position]
        point.velocities = [target_velocity]
        point.accelerations = [0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)  # 1 second to reach point

        traj_msg.points = [point]

        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f'Sent trajectory command: position={target_position:.3f}')

        self.command_count += 1


def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()

    try:
        rclpy.spin(joint_commander)
    except KeyboardInterrupt:
        pass

    joint_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Update setup.py

Add the new controller nodes to the `setup.py` file:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_package_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
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
            'joint_state_publisher = ros2_package_examples.joint_state_publisher:main',
            'joint_controller = ros2_package_examples.joint_controller:main',
            'trajectory_controller = ros2_package_examples.trajectory_controller:main',
            'joint_commander = ros2_package_examples.joint_commander:main',
        ],
    },
)
```

### Step 6: Update package.xml

Update the `package.xml` to include control message dependencies:

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
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>robot_state_publisher</depend>
  <depend>trajectory_msgs</depend>
  <depend>control_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
</xml>
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

3. In one terminal, run the joint state publisher (from Lab 3):
   ```bash
   ros2 run ros2_package_examples joint_state_publisher
   ```

4. In another terminal, run the trajectory controller:
   ```bash
   ros2 run ros2_package_examples trajectory_controller
   ```

5. In a third terminal, run the joint commander to send commands:
   ```bash
   ros2 run ros2_package_examples joint_commander
   ```

## Expected Outcome

The joint controller should:
- Receive joint state feedback from the system
- Calculate appropriate control outputs using PID control
- Send trajectory commands to control the specified joint
- Move the joint according to the commanded trajectory

## Verification

Use ROS 2 tools to verify the controller:

```bash
# Monitor the controller state
ros2 topic echo /left_elbow_state control_msgs/msg/JointTrajectoryControllerState

# Monitor commands being sent
ros2 topic echo /left_elbow_command trajectory_msgs/msg/JointTrajectory

# Check joint states
ros2 topic echo /joint_states sensor_msgs/msg/JointState

# View the TF tree to see if joint transformations are updating
ros2 run tf2_tools view_frames
```

## Real-time Control Considerations

When implementing joint controllers, consider:
- **Timing**: Control loops should run at consistent intervals
- **Latency**: Minimize delay between sensing and actuation
- **Stability**: Properly tune PID gains to avoid oscillations
- **Safety**: Implement limits and emergency stops
- **Performance**: Optimize code for real-time execution

## Troubleshooting

- If the controller is unstable, adjust PID gains (start with lower values)
- If joint doesn't move, check that joint names match between controller and URDF
- If timing is inconsistent, check for CPU load and optimize code
- Verify that all required dependencies are installed

## Next Steps

This lab introduces fundamental concepts of robot control in ROS 2. You now understand how to:
- Implement PID controllers for joint position control
- Handle trajectory following for smooth motion
- Process joint state feedback in real-time
- Integrate controllers with ROS 2 systems
- Apply control theory concepts in practical robotics applications