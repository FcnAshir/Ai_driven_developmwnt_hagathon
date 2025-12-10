---
sidebar_position: 3
---

# Lab 3: Run Isaac ROS Visual SLAM

## Overview

In this lab, you'll learn how to implement and run Visual SLAM (Simultaneous Localization and Mapping) using Isaac ROS. Visual SLAM is critical for autonomous navigation as it allows robots to understand their position in the environment while building a map simultaneously.

## Prerequisites

- Lab 1 completed (Isaac Sim installed and configured)
- Lab 2 completed (Perception pipeline basics)
- Isaac ROS Visual SLAM packages installed
- ROS 2 Humble Hawksbill installed
- Understanding of SLAM concepts

## Learning Objectives

By the end of this lab, you will be able to:
- Install and configure Isaac ROS Visual SLAM packages
- Set up a Visual SLAM pipeline for camera-based localization
- Run Visual SLAM with Isaac Sim simulation data
- Evaluate SLAM performance and accuracy
- Integrate SLAM with navigation systems

## Background

Visual SLAM (Simultaneous Localization and Mapping) is a critical capability for autonomous robots. It allows robots to:
- Estimate their position and orientation in real-time
- Build a map of the environment simultaneously
- Navigate without prior knowledge of the environment
- Plan paths based on the created map

Isaac ROS Visual SLAM provides hardware-accelerated SLAM algorithms optimized for NVIDIA GPUs, offering:
- Real-time performance with high accuracy
- Integration with other Isaac ROS packages
- Support for various camera configurations
- GPU acceleration for feature detection and tracking

## Installing Isaac ROS Visual SLAM

### Step 1: Install Isaac ROS Visual SLAM Packages

First, install the necessary Isaac ROS Visual SLAM packages:

```bash
# Update package list
sudo apt update

# Install Isaac ROS Visual SLAM packages
sudo apt install -y ros-humble-isaac-ros-visual-slam
sudo apt install -y ros-humble-isaac-ros-visual-slam-engine
sudo apt install -y ros-humble-isaac-ros-se3
sudo apt install -y ros-humble-isaac-ros-fovis
```

### Step 2: Verify Installation

Check that the Visual SLAM packages are properly installed:

```bash
# Verify packages are installed
ros2 pkg list | grep visual_slam
ros2 pkg list | grep fovis
ros2 pkg list | grep se3

# Check available launch files
find /opt/ros/humble/share -name "*slam*" -type d
```

## Setting Up Visual SLAM Pipeline

### Step 3: Create a Visual SLAM Workspace

Create a dedicated workspace for Visual SLAM:

```bash
# Create workspace for Visual SLAM
mkdir -p ~/isaac_vslam_ws/src
cd ~/isaac_vslam_ws

# Create a package for Visual SLAM configuration
cd src
ros2 pkg create --build-type ament_python visual_slam_examples
cd visual_slam_examples
```

### Step 4: Create Visual SLAM Launch File

Create a launch file for the Visual SLAM pipeline. Create `~/isaac_vslam_ws/src/visual_slam_examples/launch/visual_slam.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch arguments
    rectified_images := LaunchConfiguration('rectified_images')
    enable_rectification := LaunchConfiguration('enable_rectification')
    use_image_pyr := LaunchConfiguration('use_image_pyr')
    enable_imu_fusion := LaunchConfiguration('enable_imu_fusion')
    map_frame := LaunchConfiguration('map_frame')
    odom_frame := LaunchConfiguration('odom_frame')
    base_frame := LaunchConfiguration('base_frame')
    publish_frame := LaunchConfiguration('publish_frame')

    # Declare launch arguments
    rectified_images_arg = DeclareLaunchArgument(
        'rectified_images',
        default_value='true',
        description='Use rectified images for Visual SLAM'
    )

    enable_rectification_arg = DeclareLaunchArgument(
        'enable_rectification',
        default_value='true',
        description='Enable image rectification'
    )

    use_image_pyr_arg = DeclareLaunchArgument(
        'use_image_pyr',
        default_value='true',
        description='Use image pyramids for feature tracking'
    )

    enable_imu_fusion_arg = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='false',
        description='Enable IMU fusion for Visual SLAM'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )

    publish_frame_arg = DeclareLaunchArgument(
        'publish_frame',
        default_value='slam_map',
        description='Frame to publish SLAM map to'
    )

    # Create container for Visual SLAM nodes
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='visual_slam',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Feature Detection Node
            ComposableNode(
                package='isaac_ros_fova_stereo_image_rectifier',
                plugin='nvidia::isaac_ros::dnn_stereo::ImageRectifierNode',
                name='image_rectifier_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                    'left_topic': '/camera/left/image_raw',
                    'right_topic': '/camera/right/image_raw',
                    'left_camera_info_topic': '/camera/left/camera_info',
                    'right_camera_info_topic': '/camera/right/camera_info',
                    'left_rectified_topic': '/camera/left/image_rect',
                    'right_rectified_topic': '/camera/right/image_rect',
                }],
                condition=IfCondition(enable_rectification)
            ),

            # Visual SLAM Node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node',
                parameters=[{
                    'use_sim_time': False,
                    'enable_rectification': enable_rectification,
                    'rectified_images': rectified_images,
                    'enable_debug_mode': False,
                    'use_viz': True,
                    'publish_odom_tf': True,
                    'publish_map_tf': True,
                    'frame_id': 'oak-d_frame',
                    'base_frame': base_frame,
                    'odom_frame': odom_frame,
                    'map_frame': map_frame,
                    'publish_frame': publish_frame,
                    'use_image_pyr': use_image_pyr,
                    'enable_imu_fusion': enable_imu_fusion,
                    'gyroscope_noise_density': 0.0001,
                    'gyroscope_random_walk': 2e-05,
                    'accelerometer_noise_density': 0.001,
                    'accelerometer_random_walk': 0.0001,
                }],
                remappings=[
                    ('/visual_slam/imu', '/imu/data'),
                    ('/visual_slam/left/camera_info', '/camera/left/camera_info'),
                    ('/visual_slam/right/camera_info', '/camera/right/camera_info'),
                    ('/visual_slam/left/image_rect', '/camera/left/image_rect'),
                    ('/visual_slam/right/image_rect', '/camera/right/image_rect'),
                    ('/visual_slam/tracking/feature0', '/feature0'),
                    ('/visual_slam/tracking/feature1', '/feature1'),
                    ('/visual_slam/tracking/feature2', '/feature2'),
                    ('/visual_slam/tracking/feature3', '/feature3'),
                    ('/visual_slam/visual_odometry', '/visual_odometry'),
                    ('/visual_slam/pose_graph/optimization_result', '/optimization_result'),
                    ('/visual_slam/map_pose', '/map_pose'),
                ]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        rectified_images_arg,
        enable_rectification_arg,
        use_image_pyr_arg,
        enable_imu_fusion_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        publish_frame_arg,
        visual_slam_container,
    ])
```

### Step 5: Create Visual SLAM Configuration

Create a configuration file for Visual SLAM parameters. Create `~/isaac_vslam_ws/src/visual_slam_examples/config/vslam_params.yaml`:

```yaml
visual_slam:
  ros__parameters:
    use_sim_time: false
    enable_rectification: true
    rectified_images: true
    enable_debug_mode: false
    use_viz: true
    publish_odom_tf: true
    publish_map_tf: true
    frame_id: "oak-d_frame"
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    publish_frame: "slam_map"
    use_image_pyr: true
    enable_imu_fusion: false
    gyroscope_noise_density: 0.0001
    gyroscope_random_walk: 2.0e-05
    accelerometer_noise_density: 0.001
    accelerometer_random_walk: 0.0001
    max_num_features: 1000
    num_levels: 3
    match_ratio_threshold: 0.8
    max_reproj_distance: 2.0
    min_track_length: 3
    max_pose_graph_nodes: 1000
    min_distance: 0.1
    min_angle: 0.1
    max_pose_graph_error: 1.0
    max_num_bad_matches: 100
    max_reproj_error: 10.0
    max_desc_distance: 0.7
    max_epipolar_error: 2.0
    min_triangulation_angle: 5.0
    max_triangulation_distance: 50.0
    min_depth: 0.1
    max_depth: 50.0
```

### Step 6: Create Visualization Node

Create a simple visualization node to display SLAM results. Create `~/isaac_vslam_ws/src/visual_slam_examples/visual_slam_examples/vslam_viz_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
import numpy as np

class VslamVizNode(Node):
    def __init__(self):
        super().__init__('vslam_viz_node')

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/visual_odometry',
            self.odom_callback,
            10
        )

        self.map_pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/map_pose',
            self.map_pose_callback,
            10
        )

        # Create publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/slam_path', 10)
        self.path_pub = self.create_publisher(Odometry, '/slam_path_odom', 10)

        # Initialize variables
        self.path = []
        self.bridge = CvBridge()

        self.get_logger().info('Visual SLAM Visualization Node has been started')

    def odom_callback(self, msg):
        """Process odometry messages from Visual SLAM"""
        # Store the position for path visualization
        self.path.append((msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z))

        # Publish the path as markers
        self.publish_path_markers()

        # Publish to path topic
        self.path_pub.publish(msg)

    def map_pose_callback(self, msg):
        """Process map pose messages"""
        self.get_logger().info(f'Received map pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}')

    def publish_path_markers(self):
        """Publish path as visualization markers"""
        if len(self.path) < 2:
            return

        marker_array = MarkerArray()

        # Create path marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "slam_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.02  # Line width

        path_marker.color.r = 1.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        # Add points to the path
        for i, (x, y, z) in enumerate(self.path[-100:]):  # Only show last 100 points
            point = path_marker.points.add()
            point.x = x
            point.y = y
            point.z = z

        marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    vslam_viz_node = VslamVizNode()

    try:
        rclpy.spin(vslam_viz_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_viz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Create Package Configuration

Create the package configuration file `~/isaac_vslam_ws/src/visual_slam_examples/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'visual_slam_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visual_slam.launch.py']),
        ('share/' + package_name + '/config', ['config/vslam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Isaac ROS Visual SLAM Examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vslam_viz_node = visual_slam_examples.vslam_viz_node:main',
        ],
    },
)
```

Create the package manifest `~/isaac_vslam_ws/src/visual_slam_examples/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>visual_slam_examples</name>
  <version>0.0.1</version>
  <description>Isaac ROS Visual SLAM Examples</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>cv_bridge</depend>

  <exec_depend>isaac_ros_visual_slam</exec_depend>
  <exec_depend>isaac_ros_common</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Testing Visual SLAM with Isaac Sim

### Step 8: Build the Workspace

```bash
cd ~/isaac_vslam_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_slam_examples
source install/setup.bash
```

### Step 9: Configure Isaac Sim Scene for Visual SLAM

1. Launch Isaac Sim
2. Create a new scene with a robot that has stereo cameras
3. Set up the stereo cameras with appropriate baseline (typically 10-20cm)
4. Configure the cameras to publish to the correct topics:
   - Left camera: `/camera/left/image_rect_color` or `/camera/left/image_raw`
   - Right camera: `/camera/right/image_rect_color` or `/camera/right/image_raw`
   - Camera info for both cameras

### Step 10: Run Visual SLAM

```bash
# Terminal 1: Launch Visual SLAM
source ~/isaac_vslam_ws/install/setup.bash
ros2 launch visual_slam_examples visual_slam.launch.py

# Terminal 2: Run visualization node
source ~/isaac_vslam_ws/install/setup.bash
ros2 run visual_slam_examples vslam_viz_node

# Terminal 3: Monitor results
source /opt/ros/humble/setup.bash
# Monitor odometry
ros2 topic echo /visual_slam/visual_odometry
# Monitor map pose
ros2 topic echo /visual_slam/map_pose
# Monitor TF frames
ros2 run tf2_tools view_frames
```

## Advanced Visual SLAM Configuration

### Step 11: Adding IMU Fusion

To improve Visual SLAM accuracy, you can add IMU fusion:

```python
# Add IMU fusion node to your launch file
ComposableNode(
    package='isaac_ros_imu_fusion',
    plugin='nvidia::isaac_ros::imu_fusion::ImuFusionNode',
    name='imu_fusion_node',
    parameters=[{
        'use_sim_time': False,
        'sensor_qos': True,
        'gyroscope_noise_density': 0.0001,
        'gyroscope_random_walk': 2.0e-05,
        'accelerometer_noise_density': 0.001,
        'accelerometer_random_walk': 0.0001,
    }],
    remappings=[
        ('imu', '/imu/data'),
        ('integrated_vio', '/integrated_vio'),
    ]
)
```

### Step 12: Loop Closure Configuration

Enable loop closure for better map consistency:

```yaml
visual_slam:
  ros__parameters:
    # ... other parameters ...
    enable_loop_closure: true
    loop_closure_min_distance: 1.0
    loop_closure_min_angle: 0.5
    loop_closure_search_radius: 5.0
    loop_closure_search_angle: 1.0
    max_num_loop_closure_matches: 10
    min_num_loop_closure_matches: 5
    loop_closure_reprojection_threshold: 5.0
```

## Evaluating Visual SLAM Performance

### Step 13: Performance Metrics

Monitor these key performance metrics:

1. **Tracking Success Rate**: Percentage of frames where features are successfully tracked
2. **Map Consistency**: How well the map aligns with itself over time
3. **Drift**: Accumulation of error over distance traveled
4. **Real-time Performance**: Frame rate and processing time per frame
5. **Feature Density**: Number of reliable features being tracked

### Step 14: Using Evaluation Tools

```bash
# Monitor processing performance
ros2 run plotjuggler plotjuggler

# Monitor TF tree
ros2 run tf2_tools view_frames

# Check message rates
ros2 topic hz /visual_slam/visual_odometry
```

## Troubleshooting Common Issues

### Issue 1: Tracking Failure
**Symptoms**: Visual SLAM loses tracking frequently
**Solutions**:
- Ensure sufficient lighting in the environment
- Check for sufficient visual features (texture, not overly smooth surfaces)
- Verify camera calibration is accurate
- Reduce motion speed during initialization

### Issue 2: Drift Accumulation
**Symptoms**: Robot position estimate drifts significantly over time
**Solutions**:
- Enable loop closure detection
- Improve feature matching parameters
- Use IMU fusion for better pose estimation
- Ensure proper camera calibration

### Issue 3: High CPU/GPU Usage
**Symptoms**: System becomes unresponsive during SLAM
**Solutions**:
- Reduce image resolution
- Lower feature count parameters
- Reduce number of pyramid levels
- Optimize GPU memory usage

### Issue 4: Map Inconsistency
**Symptoms**: Map doesn't align properly when revisiting areas
**Solutions**:
- Tune loop closure parameters
- Improve feature descriptor quality
- Adjust reprojection thresholds
- Verify proper initialization

## Integration with Navigation

### Step 15: Connecting to Nav2

To use the SLAM map with Navigation2:

1. The SLAM system will publish occupancy grid maps
2. Configure Nav2 to use the SLAM map as the source
3. Set up proper coordinate frames between SLAM and Nav2

```yaml
# In your Nav2 configuration
slam_map:
  ros__parameters:
    use_sim_time: true
    topic: "/slam_map"
    qos: 10
    subscribe_to_updates: true
```

## Verification

To verify that your Visual SLAM system is working correctly:

1. Visual SLAM nodes launch without errors
2. The robot pose is published consistently
3. Map building progresses as the robot moves
4. Loop closures are detected when returning to known areas
5. TF frames (map, odom, base_link) are published correctly
6. Visualization shows the robot path and map

## Performance Optimization Tips

1. **Feature Management**: Balance feature count for tracking quality vs. performance
2. **Pyramid Levels**: Use appropriate number of image pyramid levels
3. **Image Resolution**: Optimize for your specific application needs
4. **Loop Closure**: Properly configure loop closure for large environments

## Summary

In this lab, you've successfully implemented and run Isaac ROS Visual SLAM with Isaac Sim. You've learned how to configure the SLAM pipeline, integrate it with your robot's sensors, and evaluate its performance. Visual SLAM provides critical capabilities for autonomous navigation and mapping in unknown environments.

## Next Steps

In Lab 4, you'll create a Nav2 navigation demo that uses the maps generated by your Visual SLAM system to enable autonomous navigation in Isaac Sim.