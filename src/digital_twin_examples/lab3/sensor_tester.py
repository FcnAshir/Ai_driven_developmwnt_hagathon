#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np


class SensorTester(Node):
    def __init__(self):
        super().__init__('sensor_tester')

        # Create subscribers for all sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/humanoid_robot/image',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu',
            self.imu_callback,
            10
        )

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Statistics
        self.lidar_count = 0
        self.image_count = 0
        self.imu_count = 0

        self.get_logger().info('Sensor Tester Node Started')

    def lidar_callback(self, msg):
        self.lidar_count += 1
        if self.lidar_count % 100 == 0:  # Log every 100 messages
            range_min = min(msg.ranges) if msg.ranges else float('inf')
            range_max = max(msg.ranges) if msg.ranges else 0.0
            self.get_logger().info(
                f'Lidar: {self.lidar_count} messages received. '
                f'Min range: {range_min:.2f}m, Max range: {range_max:.2f}m'
            )

    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count % 100 == 0:  # Log every 100 messages
            self.get_logger().info(
                f'Image: {self.image_count} messages received. '
                f'Size: {msg.width}x{msg.height}'
            )

    def imu_callback(self, msg):
        self.imu_count += 1
        if self.imu_count % 100 == 0:  # Log every 100 messages
            self.get_logger().info(
                f'IMU: {self.imu_count} messages received. '
                f'Linear acceleration: ({msg.linear_acceleration.x:.2f}, '
                f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    sensor_tester = SensorTester()

    try:
        rclpy.spin(sensor_tester)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()