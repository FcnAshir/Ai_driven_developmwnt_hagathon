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