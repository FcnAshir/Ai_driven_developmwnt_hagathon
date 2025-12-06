#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Initialize joint positions for a simple oscillating motion
        self.time_step = 0.0

        # Define joint names based on our humanoid URDF
        self.joint_names = [
            'neck_joint', 'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'left_hip_joint',
            'left_knee_joint', 'right_hip_joint', 'right_knee_joint'
        ]

        self.get_logger().info('Joint Controller Node Started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = []

        # Update time step
        self.time_step += 0.05

        # Generate oscillating joint positions
        for i, joint_name in enumerate(self.joint_names):
            # Different oscillation patterns for different joints
            if 'neck' in joint_name:
                # Gentle head movement
                position = 0.2 * math.sin(self.time_step)
            elif 'shoulder' in joint_name:
                # Shoulder movement
                position = 0.5 * math.sin(self.time_step + i)
            elif 'elbow' in joint_name:
                # Elbow movement
                position = 0.3 * math.sin(self.time_step * 1.5 + i)
            elif 'hip' in joint_name:
                # Hip movement
                position = 0.4 * math.sin(self.time_step * 0.8 + i)
            elif 'knee' in joint_name:
                # Knee movement
                position = 0.4 * math.sin(self.time_step * 0.8 + i + 1.5)
            else:
                # Default position
                position = 0.0

            msg.position.append(position)

        # Add timestamp and frame
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish the message
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()