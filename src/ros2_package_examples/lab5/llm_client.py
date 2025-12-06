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