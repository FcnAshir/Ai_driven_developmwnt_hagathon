#!/usr/bin/env python3
"""
Unity Bridge Node for ROS-TCP-Connector communication.
This node facilitates communication between ROS 2 and Unity.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image, LaserScan
from geometry_msgs.msg import Twist
import socket
import json
import threading
import time


class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_bridge_node')

        # Unity connection parameters
        self.unity_host = 'localhost'
        self.unity_port = 10000

        # Create socket connection to Unity
        self.unity_socket = None
        self.connect_to_unity()

        # Create subscribers for ROS topics
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/humanoid_robot/image',
            self.image_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers for Unity data
        self.unity_command_pub = self.create_publisher(
            String,
            '/unity_commands',
            10
        )

        # Timer for sending data to Unity
        self.timer = self.create_timer(0.1, self.send_data_to_unity)

        self.get_logger().info('Unity Bridge Node initialized')

    def connect_to_unity(self):
        """Establish connection to Unity via TCP"""
        try:
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.connect((self.unity_host, self.unity_port))
            self.get_logger().info(f'Connected to Unity at {self.unity_host}:{self.unity_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')

    def joint_state_callback(self, msg):
        """Handle joint state messages from ROS"""
        joint_data = {
            'type': 'joint_states',
            'data': {
                'name': list(msg.name),
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort)
            }
        }
        self.send_to_unity(joint_data)

    def image_callback(self, msg):
        """Handle image messages from ROS"""
        image_data = {
            'type': 'image',
            'data': {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'is_bigendian': msg.is_bigendian,
                'step': msg.step,
                # Note: In practice, you'd need to handle the actual image data differently
                'data_size': len(msg.data)
            }
        }
        self.send_to_unity(image_data)

    def lidar_callback(self, msg):
        """Handle LiDAR scan messages from ROS"""
        lidar_data = {
            'type': 'lidar',
            'data': {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': list(msg.ranges[:100])  # Limit ranges for performance
            }
        }
        self.send_to_unity(lidar_data)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS"""
        cmd_data = {
            'type': 'cmd_vel',
            'data': {
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z
            }
        }
        self.send_to_unity(cmd_data)

    def send_data_to_unity(self):
        """Periodically send data to Unity"""
        # This function can be used to send periodic updates to Unity
        # For now, it just sends a heartbeat
        heartbeat = {
            'type': 'heartbeat',
            'data': {
                'timestamp': time.time()
            }
        }
        self.send_to_unity(heartbeat)

    def send_to_unity(self, data):
        """Send data to Unity via TCP socket"""
        if self.unity_socket:
            try:
                json_data = json.dumps(data)
                self.unity_socket.send(json_data.encode() + b'\n')
            except Exception as e:
                self.get_logger().error(f'Failed to send data to Unity: {e}')
                # Attempt to reconnect
                self.connect_to_unity()

    def destroy_node(self):
        """Clean up resources"""
        if self.unity_socket:
            self.unity_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    unity_bridge_node = UnityBridgeNode()

    try:
        rclpy.spin(unity_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()