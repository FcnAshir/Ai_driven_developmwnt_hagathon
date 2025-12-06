#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class AdvancedJointController(Node):
    def __init__(self):
        super().__init__('advanced_joint_controller')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Initialize joint positions and velocities
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0
        }

        # Target positions for movement
        self.target_positions = self.joint_positions.copy()

        # Movement parameters
        self.move_speed = 0.02  # radians per update
        self.time_step = 0.0

        # Create a simple walking gait pattern
        self.gait_phase = 0.0
        self.gait_period = 4.0  # seconds for one gait cycle

        self.get_logger().info('Advanced Joint Controller Node Started')

    def publish_joint_states(self):
        # Update joint positions toward targets
        self.update_joint_positions()

        # Create and publish joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [0.0] * len(msg.position)  # Simplified
        msg.effort = [0.0] * len(msg.position)    # Simplified

        # Add timestamp and frame
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish the message
        self.joint_pub.publish(msg)

        # Update gait phase
        self.gait_phase += 0.05
        if self.gait_phase >= self.gait_period:
            self.gait_phase = 0.0

    def update_joint_positions(self):
        # Implement a simple walking gait
        left_leg_phase = math.sin(2 * math.pi * self.gait_phase / self.gait_period)
        right_leg_phase = math.sin(2 * math.pi * self.gait_phase / self.gait_period + math.pi)

        # Update leg joints for walking motion
        self.joint_positions['left_hip_joint'] = 0.2 * left_leg_phase
        self.joint_positions['left_knee_joint'] = 0.3 * abs(left_leg_phase)
        self.joint_positions['right_hip_joint'] = 0.2 * right_leg_phase
        self.joint_positions['right_knee_joint'] = 0.3 * abs(right_leg_phase)

        # Add some arm movement for balance
        self.joint_positions['left_shoulder_joint'] = 0.1 * right_leg_phase
        self.joint_positions['right_shoulder_joint'] = 0.1 * left_leg_phase


def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()