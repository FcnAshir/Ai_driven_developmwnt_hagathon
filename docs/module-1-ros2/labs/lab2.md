# Lab 2: Publish and Subscribe to Topics

## Objective
Understand ROS 2 communication by implementing publishers and subscribers.

## Prerequisites
- Completed Lab 1
- ROS 2 installed
- Basic Python programming knowledge

## Steps

### 1. Create publisher node
Create a new file `my_robot_package/talker_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, Robot! Message #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()

    try:
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        pass
    finally:
        talker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create subscriber node
Create a new file `my_robot_package/listener_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()

    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        pass
    finally:
        listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Update setup.py to include new executables
Edit the `setup.py` file to add the new entry points:

```python
entry_points={
    'console_scripts': [
        'simple_node = my_robot_package.simple_node:main',
        'talker_node = my_robot_package.talker_node:main',
        'listener_node = my_robot_package.listener_node:main',
    ],
},
```

### 4. Build and run the publisher and subscriber
In separate terminals:

Terminal 1 (Publisher):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_package talker_node
```

Terminal 2 (Subscriber):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_package listener_node
```

## Expected Output
- The talker node should publish messages every 0.5 seconds
- The listener node should receive and display the published messages

## Troubleshooting
- If topics don't connect, ensure both nodes are on the same ROS domain
- Use `ros2 topic list` to verify the topic exists
- Use `ros2 topic echo /robot_commands` to manually listen to the topic

## Next Steps
- Experiment with different message types
- Learn about Quality of Service (QoS) settings
- Implement services and actions for request/response communication