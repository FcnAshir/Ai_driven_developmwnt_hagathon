# Lab 5: Connect a Local LLM Agent to ROS Actions

## Objective
Integrate a local LLM for higher-level command interpretation and execution through ROS 2 actions.

## Prerequisites
- Completed previous labs (especially Lab 4)
- ROS 2 with action support
- Python packages: openai (or local LLM alternative), rclpy
- Basic understanding of ROS 2 actions

## Steps

### 1. Create an LLM agent package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python llm_robot_agent
cd llm_robot_agent
```

### 2. Create the action definition file
Create `action/RobotCommand.action`:

```
# Goal: Natural language command for the robot
string command
---
# Result: Execution status
bool success
string message
---
# Feedback: Execution progress
string current_action
float64 progress
```

### 3. Create the action server (robot executor)
Create `llm_robot_agent/robot_executor_server.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from llm_robot_agent.action import RobotCommand
import time
import threading


class RobotExecutorServer(Node):
    def __init__(self):
        super().__init__('robot_executor_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            RobotCommand,
            'robot_command',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Robot state simulation
        self.current_task = None
        self.is_executing = False

        self.get_logger().info('Robot Executor Server Started')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request: {goal_request.command}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        feedback_msg = RobotCommand.Feedback()
        feedback_msg.current_action = 'Processing command'
        feedback_msg.progress = 0.0

        command = goal_handle.request.command.lower()

        # Simulate processing time
        time.sleep(0.5)
        feedback_msg.progress = 0.2
        goal_handle.publish_feedback(feedback_msg)

        # Parse and execute the command
        success = self.execute_robot_command(command, goal_handle, feedback_msg)

        if success:
            goal_handle.succeed()
            result = RobotCommand.Result()
            result.success = True
            result.message = f'Command "{command}" executed successfully'
            self.get_logger().info(f'Result: {result.message}')
            return result
        else:
            goal_handle.abort()
            result = RobotCommand.Result()
            result.success = False
            result.message = f'Failed to execute command: {command}'
            self.get_logger().info(f'Result: {result.message}')
            return result

    def execute_robot_command(self, command, goal_handle, feedback_msg):
        """Execute a specific robot command."""
        try:
            if 'move forward' in command:
                feedback_msg.current_action = 'Moving forward'
                goal_handle.publish_feedback(feedback_msg)
                self.simulate_movement('forward', 2.0, goal_handle, feedback_msg)
            elif 'turn left' in command:
                feedback_msg.current_action = 'Turning left'
                goal_handle.publish_feedback(feedback_msg)
                self.simulate_movement('left', 1.0, goal_handle, feedback_msg)
            elif 'turn right' in command:
                feedback_msg.current_action = 'Turning right'
                goal_handle.publish_feedback(feedback_msg)
                self.simulate_movement('right', 1.0, goal_handle, feedback_msg)
            elif 'pick up' in command or 'grasp' in command:
                feedback_msg.current_action = 'Picking up object'
                goal_handle.publish_feedback(feedback_msg)
                self.simulate_grasping(goal_handle, feedback_msg)
            elif 'wave' in command:
                feedback_msg.current_action = 'Waving'
                goal_handle.publish_feedback(feedback_msg)
                self.simulate_waving(goal_handle, feedback_msg)
            else:
                feedback_msg.current_action = 'Unknown command, performing default action'
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1.0)
                feedback_msg.progress = 1.0
                goal_handle.publish_feedback(feedback_msg)
                return True

            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)
            return True
        except Exception as e:
            self.get_logger().error(f'Error executing command: {str(e)}')
            return False

    def simulate_movement(self, direction, duration, goal_handle, feedback_msg):
        """Simulate robot movement."""
        steps = int(duration / 0.1)
        for i in range(steps):
            progress = 0.2 + (0.6 * i / steps)
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # Final 20% for completion
        time.sleep(0.2)
        feedback_msg.progress = 1.0

    def simulate_grasping(self, goal_handle, feedback_msg):
        """Simulate robot grasping action."""
        time.sleep(0.5)  # Move to object
        feedback_msg.progress = 0.5
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(0.3)  # Grasp
        feedback_msg.progress = 0.8
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(0.2)  # Lift
        feedback_msg.progress = 1.0

    def simulate_waving(self, goal_handle, feedback_msg):
        """Simulate robot waving action."""
        for i in range(3):
            time.sleep(0.3)
            progress = 0.2 + (0.6 * i / 3)
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)

        time.sleep(0.2)
        feedback_msg.progress = 1.0


def main(args=None):
    rclpy.init(args=args)
    robot_executor_server = RobotExecutorServer()

    try:
        rclpy.spin(robot_executor_server)
    except KeyboardInterrupt:
        pass
    finally:
        robot_executor_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Create the LLM client node
Create `llm_robot_agent/llm_client.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from llm_robot_agent.action import RobotCommand
import time


class LLMClient(Node):
    def __init__(self):
        super().__init__('llm_client')
        self._action_client = ActionClient(self, RobotCommand, 'robot_command')

    def send_goal(self, command):
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = RobotCommand.Goal()
        goal_msg.command = command

        # Send the goal
        self.get_logger().info(f'Sending goal: {command}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Set a callback for when the goal is accepted
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: {feedback.current_action} - '
            f'{feedback.progress:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    # Example commands to test
    commands = [
        "Move forward 2 meters",
        "Turn left 90 degrees",
        "Pick up the red cube",
        "Wave to the person"
    ]

    for command in commands:
        client = LLMClient()

        # Wait for a moment before sending the next command
        time.sleep(1)

        client.send_goal(command)

        # Spin to process callbacks
        rclpy.spin(client)


if __name__ == '__main__':
    main()
```

### 5. Create a simple LLM interface node (simulated)
Create `llm_robot_agent/llm_interface.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llm_robot_agent.action import RobotCommand
from rclpy.action import ActionClient


class LLMInterface(Node):
    def __init__(self):
        super().__init__('llm_interface')

        # Create action client for robot commands
        self._action_client = ActionClient(self, RobotCommand, 'robot_command')

        # Create subscriber for natural language commands
        self.subscription = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        self.get_logger().info('LLM Interface Node Started')

    def command_callback(self, msg):
        """Process natural language command and send to robot."""
        command = msg.data
        self.get_logger().info(f'Received natural language command: {command}')

        # Send command to robot executor
        self.send_robot_command(command)

    def send_robot_command(self, command):
        """Send command to robot via action server."""
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = RobotCommand.Goal()
        goal_msg.command = command

        # Send goal asynchronously
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Robot command accepted')
            # Get result
            goal_handle.get_result_async().add_done_callback(
                self.get_result_callback
            )
        else:
            self.get_logger().info('Robot command rejected')

    def get_result_callback(self, future):
        """Handle result from robot."""
        result = future.result().result
        self.get_logger().info(f'Robot command result: {result.message}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback from robot."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Robot feedback: {feedback.current_action} '
            f'({feedback.progress:.1%} complete)'
        )


def main(args=None):
    rclpy.init(args=args)
    llm_interface = LLMInterface()

    try:
        rclpy.spin(llm_interface)
    except KeyboardInterrupt:
        pass
    finally:
        llm_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6. Update setup.py to include executables
Edit `setup.py`:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'llm_robot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include action files
        (os.path.join('share', package_name, 'action'),
         ['action/RobotCommand.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='LLM agent for robot command interpretation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_executor_server = llm_robot_agent.robot_executor_server:main',
            'llm_client = llm_robot_agent.llm_client:main',
            'llm_interface = llm_robot_agent.llm_interface:main',
        ],
    },
)
```

### 7. Update package.xml to include action dependencies
Edit `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>llm_robot_agent</name>
  <version>0.0.0</version>
  <description>LLM agent for robot command interpretation</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <exec_depend>ros2action</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  <depend>action_msgs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 8. Build and run the LLM integration
```bash
cd ~/ros2_ws
colcon build --packages-select llm_robot_agent
source install/setup.bash

# Terminal 1: Start the robot executor server
ros2 run llm_robot_agent robot_executor_server

# Terminal 2: Send commands via the LLM interface
# First, publish a command to the topic
ros2 topic pub /natural_language_commands std_msgs/String "data: 'Move forward 2 meters'"

# Or use the client directly
ros2 run llm_robot_agent llm_client
```

## Expected Output
- The robot executor server should accept and execute commands
- The LLM interface should process natural language commands and send them to the robot
- You should see feedback messages showing the robot's progress on tasks

## Troubleshooting
- If the action server doesn't respond, check that it's running and the action name matches
- Verify that the action definition files are properly built
- Check that the command format matches what the executor expects

## Next Steps
- Integrate with a real LLM API (like OpenAI GPT)
- Add more sophisticated natural language processing
- Implement error handling and recovery behaviors
- Connect to real robot hardware for physical execution