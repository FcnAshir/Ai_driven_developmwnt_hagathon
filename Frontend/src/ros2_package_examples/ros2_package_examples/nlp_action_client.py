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