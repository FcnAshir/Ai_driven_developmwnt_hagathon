# Lab 5: Create a Digital Twin Scene with Two-Way ROS Communication

## Objective
Establish a functional digital twin with real-time ROS 2 communication between simulation and control systems.

## Prerequisites
- Completed all previous labs in Module 2
- Gazebo and Unity properly set up
- ROS 2 communication infrastructure
- Basic understanding of digital twin concepts

## Steps

### 1. Create a digital twin manager package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python digital_twin_manager
cd digital_twin_manager
```

### 2. Create the digital twin manager node
Create `digital_twin_manager/digital_twin_manager.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Time
import json
import threading
import time
from collections import deque


class DigitalTwinManager(Node):
    def __init__(self):
        super().__init__('digital_twin_manager')

        # Publishers for simulation and Unity
        self.gazebo_cmd_pub = self.create_publisher(
            JointState, '/joint_commands', 10
        )
        self.unity_state_pub = self.create_publisher(
            String, 'unity/robot_state', 10
        )
        self.digital_twin_status_pub = self.create_publisher(
            Bool, 'digital_twin/status', 10
        )

        # Subscribers for sensor data from Gazebo
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu', self.imu_callback, 10
        )

        # Subscribers for commands from Unity or external systems
        self.unity_cmd_sub = self.create_subscription(
            String, 'unity/robot_commands', self.unity_command_callback, 10
        )
        self.external_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.external_command_callback, 10
        )

        # Data storage
        self.current_joint_states = JointState()
        self.lidar_data = None
        self.imu_data = None
        self.unity_commands = deque(maxlen=10)
        self.external_commands = deque(maxlen=10)

        # Synchronization parameters
        self.update_rate = 50  # Hz
        self.timer = self.create_timer(1.0/self.update_rate, self.sync_callback)

        # Status tracking
        self.is_synchronized = True
        self.last_sync_time = self.get_clock().now()

        # Statistics
        self.sync_count = 0
        self.command_count = 0

        self.get_logger().info('Digital Twin Manager Node Started')

    def joint_state_callback(self, msg):
        """Handle joint state updates from Gazebo."""
        self.current_joint_states = msg
        self.current_joint_states.header.stamp = self.get_clock().now().to_msg()

    def lidar_callback(self, msg):
        """Handle LiDAR data from Gazebo."""
        self.lidar_data = msg

    def imu_callback(self, msg):
        """Handle IMU data from Gazebo."""
        self.imu_data = msg

    def unity_command_callback(self, msg):
        """Handle commands from Unity."""
        try:
            command_data = json.loads(msg.data)
            self.unity_commands.append(command_data)
            self.command_count += 1

            # Process the command
            self.process_unity_command(command_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in Unity command')

    def external_command_callback(self, msg):
        """Handle external commands (e.g., from navigation stack)."""
        command = {
            'type': 'velocity_command',
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.external_commands.append(command)
        self.command_count += 1

        # Forward to simulation
        self.execute_external_command(msg)

    def process_unity_command(self, command_data):
        """Process commands received from Unity."""
        cmd_type = command_data.get('type', 'unknown')

        if cmd_type == 'joint_position':
            # Create joint command for Gazebo
            joint_cmd = JointState()
            joint_cmd.name = command_data.get('joint_names', [])
            joint_cmd.position = command_data.get('positions', [])
            joint_cmd.header.stamp = self.get_clock().now().to_msg()

            # Publish to Gazebo
            self.gazebo_cmd_pub.publish(joint_cmd)

        elif cmd_type == 'move_to':
            # Handle move-to commands
            target_pose = command_data.get('pose', {})
            self.get_logger().info(f'Moving to pose: {target_pose}')

        elif cmd_type == 'gripper_control':
            # Handle gripper commands
            gripper_cmd = command_data.get('command', 'open')
            self.get_logger().info(f'Gripper command: {gripper_cmd}')

    def execute_external_command(self, cmd_msg):
        """Execute external commands in the simulation."""
        # Convert Twist command to appropriate joint commands
        # This would depend on your robot's specific kinematics
        joint_cmd = JointState()
        joint_cmd.name = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']

        # Simple example: convert linear/angular velocity to leg movements
        linear_vel = cmd_msg.linear.x
        angular_vel = cmd_msg.angular.z

        # Calculate joint positions based on desired velocity
        # This is a simplified example - real implementation would use inverse kinematics
        joint_positions = [
            linear_vel * 0.1 + angular_vel * 0.05,  # left_hip
            linear_vel * 0.1 - angular_vel * 0.05,  # right_hip
            linear_vel * 0.05,                       # left_knee
            linear_vel * 0.05                        # right_knee
        ]

        joint_cmd.position = joint_positions
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Publish to simulation
        self.gazebo_cmd_pub.publish(joint_cmd)

    def sync_callback(self):
        """Synchronize data between real and simulated systems."""
        self.sync_count += 1

        # Create synchronized state message for Unity
        twin_state = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'sync_count': self.sync_count,
            'joint_states': {
                'names': list(self.current_joint_states.name),
                'positions': list(self.current_joint_states.position),
                'velocities': list(self.current_joint_states.velocity),
                'effort': list(self.current_joint_states.effort)
            },
            'sensors': {
                'lidar': self.pack_lidar_data(),
                'imu': self.pack_imu_data()
            },
            'status': {
                'is_synchronized': self.is_synchronized,
                'command_count': self.command_count,
                'update_rate': self.update_rate
            }
        }

        # Publish state to Unity
        unity_msg = String()
        unity_msg.data = json.dumps(twin_state, separators=(',', ':'))
        self.unity_state_pub.publish(unity_msg)

        # Publish status
        status_msg = Bool()
        status_msg.data = self.is_synchronized
        self.digital_twin_status_pub.publish(status_msg)

        # Log sync info periodically
        if self.sync_count % (self.update_rate * 2) == 0:  # Every 2 seconds
            self.get_logger().info(
                f'Digital Twin Sync: {self.sync_count} cycles, '
                f'{self.command_count} commands processed'
            )

    def pack_lidar_data(self):
        """Pack LiDAR data for transmission."""
        if self.lidar_data is None:
            return None

        # For efficiency, only send key data points
        return {
            'angle_min': self.lidar_data.angle_min,
            'angle_max': self.lidar_data.angle_max,
            'angle_increment': self.lidar_data.angle_increment,
            'range_min': self.lidar_data.range_min,
            'range_max': self.lidar_data.range_max,
            'ranges': self.lidar_data.ranges[::10]  # Downsample by 10
        }

    def pack_imu_data(self):
        """Pack IMU data for transmission."""
        if self.imu_data is None:
            return None

        return {
            'orientation': {
                'x': self.imu_data.orientation.x,
                'y': self.imu_data.orientation.y,
                'z': self.imu_data.orientation.z,
                'w': self.imu_data.orientation.w
            },
            'angular_velocity': {
                'x': self.imu_data.angular_velocity.x,
                'y': self.imu_data.angular_velocity.y,
                'z': self.imu_data.angular_velocity.z
            },
            'linear_acceleration': {
                'x': self.imu_data.linear_acceleration.x,
                'y': self.imu_data.linear_acceleration.y,
                'z': self.imu_data.linear_acceleration.z
            }
        }

    def validate_synchronization(self):
        """Validate that real and simulated systems are properly synchronized."""
        # Check timing consistency
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_sync_time).nanoseconds / 1e9

        if time_diff > 2.0 / self.update_rate:  # Allow 2x expected interval
            self.get_logger().warn('Synchronization delay detected')
            self.is_synchronized = False
        else:
            self.is_synchronized = True

        self.last_sync_time = current_time


def main(args=None):
    rclpy.init(args=args)
    digital_twin_manager = DigitalTwinManager()

    try:
        rclpy.spin(digital_twin_manager)
    except KeyboardInterrupt:
        pass
    finally:
        digital_twin_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Create a digital twin monitor node
Create `digital_twin_manager/digital_twin_monitor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, LaserScan, Imu
import json
import time


class DigitalTwinMonitor(Node):
    def __init__(self):
        super().__init__('digital_twin_monitor')

        # Subscribers for digital twin data
        self.unity_state_sub = self.create_subscription(
            String, 'unity/robot_state', self.unity_state_callback, 10
        )
        self.status_sub = self.create_subscription(
            Bool, 'digital_twin/status', self.status_callback, 10
        )

        # Publishers for monitoring
        self.monitor_pub = self.create_publisher(
            String, 'digital_twin/monitor', 10
        )

        # Statistics
        self.state_count = 0
        self.status_count = 0
        self.last_state_time = None
        self.synchronization_issues = 0

        # Timer for periodic monitoring
        self.timer = self.create_timer(1.0, self.monitor_callback)

        self.get_logger().info('Digital Twin Monitor Node Started')

    def unity_state_callback(self, msg):
        """Process state updates from the digital twin."""
        try:
            state_data = json.loads(msg.data)
            self.state_count += 1
            self.last_state_time = time.time()

            # Check for synchronization issues
            if not state_data.get('status', {}).get('is_synchronized', True):
                self.synchronization_issues += 1
                self.get_logger().warn('Digital twin synchronization issue detected')

            # Log key metrics periodically
            if self.state_count % 50 == 0:  # Every 50 states
                self.get_logger().info(
                    f'Digital Twin Monitor: {self.state_count} states received, '
                    f'{self.synchronization_issues} sync issues'
                )

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in digital twin state')

    def status_callback(self, msg):
        """Process status updates from the digital twin."""
        self.status_count += 1

        if not msg.data:  # Not synchronized
            self.get_logger().warn('Digital twin is not synchronized')

    def monitor_callback(self):
        """Perform periodic monitoring tasks."""
        # Calculate state rate
        if self.last_state_time is not None:
            current_time = time.time()
            time_diff = current_time - self.last_state_time

            if time_diff > 2.0:  # No state updates in 2 seconds
                self.get_logger().warn('Digital twin state not updating')

        # Publish monitoring summary
        monitor_data = {
            'timestamp': time.time(),
            'state_count': self.state_count,
            'status_count': self.status_count,
            'synchronization_issues': self.synchronization_issues,
            'health_status': 'OK' if self.synchronization_issues == 0 else 'WARNING'
        }

        monitor_msg = String()
        monitor_msg.data = json.dumps(monitor_data)
        self.monitor_pub.publish(monitor_msg)

    def get_system_health(self):
        """Get overall system health status."""
        return {
            'state_rate': self.state_count,
            'sync_issues': self.synchronization_issues,
            'status_rate': self.status_count
        }


def main(args=None):
    rclpy.init(args=args)
    digital_twin_monitor = DigitalTwinMonitor()

    try:
        rclpy.spin(digital_twin_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        digital_twin_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Create a launch file for the complete digital twin system
Create `launch/digital_twin_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='50',
        description='Update rate for digital twin synchronization'
    )

    update_rate = LaunchConfiguration('update_rate')

    return LaunchDescription([
        update_rate_arg,

        # Digital Twin Manager
        Node(
            package='digital_twin_manager',
            executable='digital_twin_manager',
            name='digital_twin_manager',
            output='screen',
            parameters=[
                {'update_rate': update_rate}
            ]
        ),

        # Digital Twin Monitor
        Node(
            package='digital_twin_manager',
            executable='digital_twin_monitor',
            name='digital_twin_monitor',
            output='screen'
        ),

        # Additional nodes for complete system
        Node(
            package='rviz2',
            executable='rviz2',
            name='digital_twin_rviz',
            arguments=['-d', 'path/to/digital_twin_config.rviz'],
            output='screen'
        )
    ])
```

### 5. Create a comprehensive digital twin configuration
Create `config/digital_twin_config.yaml`:

```yaml
digital_twin_manager:
  ros__parameters:
    # Synchronization parameters
    update_rate: 50.0
    max_sync_delay: 0.1  # seconds
    enable_validation: true

    # Communication parameters
    unity_bridge_enabled: true
    gazebo_bridge_enabled: true
    external_control_enabled: true

    # Data filtering
    joint_state_filter: true
    sensor_data_downsample: 10  # Send every 10th sensor reading

    # Logging
    enable_detailed_logging: false
    log_statistics_interval: 5.0  # seconds

digital_twin_monitor:
  ros__parameters:
    # Monitoring parameters
    status_check_rate: 1.0  # Hz
    warning_threshold: 5    # number of sync issues before warning
    health_check_interval: 10.0  # seconds
```

### 6. Create a digital twin test script
Create `test/digital_twin_test.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time


class DigitalTwinTest(Node):
    def __init__(self):
        super().__init__('digital_twin_test')

        # Publishers for testing
        self.unity_cmd_pub = self.create_publisher(
            String, 'unity/robot_commands', 10
        )
        self.external_cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # Timer for sending test commands
        self.timer = self.create_timer(2.0, self.send_test_commands)
        self.command_sequence = 0

        self.get_logger().info('Digital Twin Test Node Started')

    def send_test_commands(self):
        """Send test commands to verify digital twin functionality."""
        if self.command_sequence == 0:
            # Test joint position command
            cmd_data = {
                'type': 'joint_position',
                'joint_names': ['left_shoulder_joint', 'right_shoulder_joint'],
                'positions': [0.5, -0.5],
                'timestamp': time.time()
            }
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.unity_cmd_pub.publish(cmd_msg)
            self.get_logger().info('Sent joint position command')

        elif self.command_sequence == 1:
            # Test velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5  # Move forward
            cmd_msg.angular.z = 0.2  # Turn slightly
            self.external_cmd_pub.publish(cmd_msg)
            self.get_logger().info('Sent velocity command')

        elif self.command_sequence == 2:
            # Test gripper command
            cmd_data = {
                'type': 'gripper_control',
                'command': 'close',
                'timestamp': time.time()
            }
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.unity_cmd_pub.publish(cmd_msg)
            self.get_logger().info('Sent gripper command')

        self.command_sequence = (self.command_sequence + 1) % 3


def main(args=None):
    rclpy.init(args=args)
    digital_twin_test = DigitalTwinTest()

    try:
        rclpy.spin(digital_twin_test)
    except KeyboardInterrupt:
        pass
    finally:
        digital_twin_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 7. Update setup.py for the digital twin package
Edit `setup.py`:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'digital_twin_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch and config files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Digital twin manager for ROS 2 integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'digital_twin_manager = digital_twin_manager.digital_twin_manager:main',
            'digital_twin_monitor = digital_twin_manager.digital_twin_monitor:main',
            'digital_twin_test = test.digital_twin_test:main',
        ],
    },
)
```

### 8. Update package.xml
Edit `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>digital_twin_manager</name>
  <version>0.0.0</version>
  <description>Digital twin manager for ROS 2 integration</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
</file>
```

### 9. Build and test the digital twin system
```bash
cd ~/ros2_ws
colcon build --packages-select digital_twin_manager
source install/setup.bash

# Terminal 1: Start the digital twin manager
ros2 run digital_twin_manager digital_twin_manager

# Terminal 2: Start the monitor
ros2 run digital_twin_manager digital_twin_monitor

# Terminal 3: Run the test
ros2 run digital_twin_manager digital_twin_test

# Terminal 4: Monitor topics
ros2 topic echo /unity/robot_state
```

## Expected Output
- Digital twin manager running with synchronized data flow
- Real-time communication between Gazebo simulation and Unity visualization
- Commands from Unity affecting the Gazebo simulation
- Commands from external systems (like navigation) affecting the simulation
- Monitor node tracking system health and synchronization status

## Troubleshooting
- If synchronization fails, check network connectivity and ROS domain settings
- If commands aren't processed, verify topic names and message formats
- If performance is poor, adjust the update rate and data filtering parameters

## Next Steps
- Integrate with real robot hardware for hybrid digital twin
- Add machine learning components for predictive maintenance
- Implement advanced visualization in Unity
- Add cloud connectivity for remote monitoring and control