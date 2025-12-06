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