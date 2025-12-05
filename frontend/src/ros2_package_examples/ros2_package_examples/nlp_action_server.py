#!/usr/bin/env python3
# nlp_action_server.py

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
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