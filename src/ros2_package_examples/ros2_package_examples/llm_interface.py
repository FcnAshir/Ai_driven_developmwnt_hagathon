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