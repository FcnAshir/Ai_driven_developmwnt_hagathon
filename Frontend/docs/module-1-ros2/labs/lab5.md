---
sidebar_position: 5
---

# Lab 5: Connect a Local LLM Agent to ROS Actions

## Objective

In this lab, you will learn how to integrate a local Large Language Model (LLM) agent with ROS 2 actions to enable high-level command interpretation and task execution. You'll create an action server that can receive natural language commands, process them through an LLM, and execute appropriate robotic actions.

## Prerequisites

- Understanding of ROS 2 actions and services
- Basic knowledge of natural language processing concepts
- Completion of previous labs
- Basic understanding of AI/ML concepts

## Learning Outcomes

By completing this lab, you will be able to:
- Create and implement ROS 2 actions for high-level task execution
- Integrate external AI services with ROS 2 systems
- Process natural language commands for robotic tasks
- Design action-based interfaces for intelligent robotic systems
- Handle complex task planning and execution in ROS 2

## Tasks

1. Create a custom action definition for natural language command execution
2. Implement an action server that processes natural language commands
3. Integrate a local LLM (simulated) to interpret commands
4. Create an action client to send commands to the server
5. Test the system with various natural language commands

## Steps

### Step 1: Define the Natural Language Command Action

Create a custom action definition for processing natural language commands. First, create the action directory:

```bash
mkdir -p src/ros2_package_examples/action
```

Create the file `src/ros2_package_examples/action/NaturalLanguageCommand.action`:

```
# NaturalLanguageCommand.action
# Action for processing natural language commands

# Goal: Natural language command to execute
string command
string context  # Additional context for the command

---
# Result: Outcome of command processing
bool success
string message
string executed_action
float32 execution_time

---
# Feedback: Progress of command processing
string status
string current_task
float32 progress  # 0.0 to 1.0
```

### Step 2: Update package.xml for Action Support

Update the `package.xml` file to include action dependencies:

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

  <buildtool_depend>ament_python</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 3: Update setup.py for Action Support

Update the `setup.py` file to include action files:

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
        # Include action files
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
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
            'nlp_action_server = ros2_package_examples.nlp_action_server:main',
            'nlp_action_client = ros2_package_examples.nlp_action_client:main',
        ],
    },
)
```

### Step 4: Create the Action Server Node

Create an action server that receives natural language commands, processes them, and executes appropriate robotic actions. Create the file `src/ros2_package_examples/ros2_package_examples/nlp_action_server.py`:

```python
#!/usr/bin/env python3
# nlp_action_server.py

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class NLActionServer(Node):

    def __init__(self):
        super().__init__('nlp_action_server')

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_commands', 10)

        # Subscribers for robot state
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.current_joint_positions = {}
        self.is_executing = False

        self.get_logger().info('NLP Action Server initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def execute_command(self, command, context=""):
        """Parse natural language command and execute appropriate action"""
        try:
            # Simple command parsing - in a real system, this would use NLP/LLM
            command_lower = command.lower()
            context_lower = context.lower() if context else ""

            self.get_logger().info(f'Processing command: {command_lower}')

            # Parse command and determine action
            if "move" in command_lower or "go" in command_lower:
                if "forward" in command_lower or "ahead" in command_lower:
                    return self.execute_move_command("forward")
                elif "backward" in command_lower or "back" in command_lower:
                    return self.execute_move_command("backward")
                elif "left" in command_lower:
                    return self.execute_move_command("left")
                elif "right" in command_lower:
                    return self.execute_move_command("right")
                else:
                    return self.execute_move_command("forward")  # default

            elif "turn" in command_lower or "rotate" in command_lower:
                if "left" in command_lower:
                    return self.execute_turn_command("left")
                elif "right" in command_lower:
                    return self.execute_turn_command("right")
                else:
                    return self.execute_turn_command("left")  # default

            elif "arm" in command_lower or "elbow" in command_lower or "joint" in command_lower:
                if "raise" in command_lower or "up" in command_lower:
                    return self.execute_joint_command("raise_arm")
                elif "lower" in command_lower or "down" in command_lower:
                    return self.execute_joint_command("lower_arm")
                elif "wave" in command_lower:
                    return self.execute_joint_command("wave")

            elif "stop" in command_lower or "halt" in command_lower:
                return self.execute_stop_command()

            else:
                # Unknown command
                return {
                    'success': False,
                    'action': 'unknown',
                    'error': f'Unknown command: {command}'
                }

        except Exception as e:
            return {
                'success': False,
                'action': 'error',
                'error': f'Command parsing error: {str(e)}'
            }

    def execute_move_command(self, direction):
        """Execute movement commands"""
        self.get_logger().info(f'Executing move command: {direction}')

        # Create velocity command
        cmd_msg = Twist()

        if direction == "forward":
            cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.0
        elif direction == "backward":
            cmd_msg.linear.x = -0.5  # Move backward at 0.5 m/s
            cmd_msg.angular.z = 0.0
        elif direction == "left":
            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.5  # Move left
            cmd_msg.angular.z = 0.0
        elif direction == "right":
            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = -0.5  # Move right
            cmd_msg.angular.z = 0.0

        # Publish command for 2 seconds
        for _ in range(20):  # 2 seconds at 10Hz
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.1)

        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return {
            'success': True,
            'action': f'move_{direction}',
            'error': None
        }

    def execute_turn_command(self, direction):
        """Execute turning commands"""
        self.get_logger().info(f'Executing turn command: {direction}')

        cmd_msg = Twist()

        if direction == "left":
            cmd_msg.angular.z = 0.5  # Turn left at 0.5 rad/s
        elif direction == "right":
            cmd_msg.angular.z = -0.5  # Turn right at 0.5 rad/s

        # Turn for 1 second
        for _ in range(10):  # 1 second at 10Hz
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.1)

        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return {
            'success': True,
            'action': f'turn_{direction}',
            'error': None
        }

    def execute_joint_command(self, action):
        """Execute joint commands"""
        self.get_logger().info(f'Executing joint command: {action}')

        if action == "raise_arm":
            # Create trajectory to raise the left arm
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['left_shoulder', 'left_elbow']

            point = JointTrajectoryPoint()
            point.positions = [0.5, 1.0]  # Raise shoulder and elbow
            point.velocities = [0.0, 0.0]
            point.accelerations = [0.0, 0.0]
            point.time_from_start = Duration(sec=2, nanosec=0)

            traj_msg.points = [point]
            self.joint_cmd_pub.publish(traj_msg)

        elif action == "lower_arm":
            # Create trajectory to lower the left arm
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['left_shoulder', 'left_elbow']

            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0]  # Lower shoulder and elbow
            point.velocities = [0.0, 0.0]
            point.accelerations = [0.0, 0.0]
            point.time_from_start = Duration(sec=2, nanosec=0)

            traj_msg.points = [point]
            self.joint_cmd_pub.publish(traj_msg)

        elif action == "wave":
            # Create trajectory for waving motion
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['left_shoulder', 'left_elbow']

            # Wave motion with multiple points
            points = []

            # Point 1: Initial position
            point1 = JointTrajectoryPoint()
            point1.positions = [0.0, 0.0]
            point1.velocities = [0.0, 0.0]
            point1.accelerations = [0.0, 0.0]
            point1.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
            points.append(point1)

            # Point 2: Raise elbow
            point2 = JointTrajectoryPoint()
            point2.positions = [0.0, 1.0]
            point2.velocities = [0.0, 0.0]
            point2.accelerations = [0.0, 0.0]
            point2.time_from_start = Duration(sec=1, nanosec=0)  # 1.0s
            points.append(point2)

            # Point 3: Lower elbow
            point3 = JointTrajectoryPoint()
            point3.positions = [0.0, 0.0]
            point3.velocities = [0.0, 0.0]
            point3.accelerations = [0.0, 0.0]
            point3.time_from_start = Duration(sec=1, nanosec=500000000)  # 1.5s
            points.append(point3)

            traj_msg.points = points
            self.joint_cmd_pub.publish(traj_msg)

        # Wait for trajectory to complete
        time.sleep(2.0)

        return {
            'success': True,
            'action': action,
            'error': None
        }

    def execute_stop_command(self):
        """Execute stop command"""
        self.get_logger().info('Executing stop command')

        # Stop all movement
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return {
            'success': True,
            'action': 'stop',
            'error': None
        }


def main(args=None):
    rclpy.init(args=args)

    nlp_server = NLActionServer()

    try:
        rclpy.spin(nlp_server)
    except KeyboardInterrupt:
        pass
    finally:
        nlp_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Create the Action Client Node

Create an action client to send natural language commands to the server. Create the file `src/ros2_package_examples/ros2_package_examples/nlp_action_client.py`:

```python
#!/usr/bin/env python3
# nlp_action_client.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time


class NLActionClient(Node):

    def __init__(self):
        super().__init__('nlp_action_client')

        # Create action client
        # Note: In a real implementation, this would use the actual action definition
        # For this example, we'll simulate the action client functionality
        self.get_logger().info('NLP Action Client initialized')

    def send_command(self, command, context=""):
        """Send a natural language command to the server"""
        self.get_logger().info(f'Sending command: "{command}" with context: "{context}"')

        # In a real implementation, this would create and send an action goal
        # For this example, we'll simulate the process
        result = self.simulate_command_execution(command, context)

        return result

    def simulate_command_execution(self, command, context):
        """Simulate command execution (in real system, this would communicate with action server)"""
        self.get_logger().info(f'Simulating execution of: {command}')

        # Simulate processing time
        time.sleep(1.0)

        # Return a simulated result
        return {
            'success': True,
            'message': f'Successfully processed: {command}',
            'executed_action': 'simulated_action',
            'execution_time': 1.0
        }


def main(args=None):
    rclpy.init(args=args)

    client = NLActionClient()

    # Example commands to test
    test_commands = [
        "Move forward",
        "Turn left",
        "Raise your left arm",
        "Stop moving"
    ]

    for cmd in test_commands:
        result = client.send_command(cmd)
        client.get_logger().info(f'Command result: {result}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 6: Build and Run

1. Build your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_package_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. In one terminal, run the action server:
   ```bash
   ros2 run ros2_package_examples nlp_action_server
   ```

4. In another terminal, run the action client:
   ```bash
   ros2 run ros2_package_examples nlp_action_client
   ```

## Expected Outcome

The system should:
- Accept natural language commands through the action client
- Parse the commands and execute appropriate robotic actions
- Handle movement, joint control, and other robot commands
- Provide feedback on command execution status

## Integration with Real LLM

In a real implementation, you would replace the simple command parsing with:
- Integration with local LLM APIs (like Ollama, Llama.cpp, etc.)
- Natural language understanding capabilities
- Context awareness and memory
- Complex task planning and execution

## Verification

Use ROS 2 tools to verify the system:

```bash
# List available actions
ros2 action list

# Check action definition
ros2 action typesupport show ros2_package_examples/action/NaturalLanguageCommand

# Monitor robot movement topics
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist

# Monitor joint command topics
ros2 topic echo /joint_commands trajectory_msgs/msg/JointTrajectory
```

## Troubleshooting

- If action definitions don't work, ensure rosidl dependencies are properly configured
- If commands aren't processed, check that the action server is running
- If robot doesn't move, verify that command topics match your robot's expected topics
- Check that all required dependencies are installed

## Next Steps

This lab demonstrates the integration of AI/ML capabilities with ROS 2 systems. You now understand how to:
- Create custom action definitions for high-level commands
- Implement action servers that can process complex inputs
- Integrate external AI systems with robotic platforms
- Design intelligent interfaces for robotic systems
- Handle complex task planning and execution in ROS 2