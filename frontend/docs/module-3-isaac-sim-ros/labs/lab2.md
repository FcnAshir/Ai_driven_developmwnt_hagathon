---
sidebar_position: 2
---

# Lab 2: Build a Perception Pipeline

## Overview

In this lab, you'll learn how to build a perception pipeline using Isaac ROS, NVIDIA's hardware-accelerated perception packages. You'll create a complete pipeline that includes object detection, pose estimation, and sensor processing using GPU acceleration.

## Prerequisites

- Lab 1 completed (Isaac Sim installed and configured)
- NVIDIA RTX GPU with CUDA support
- Isaac Sim and Isaac ROS extensions enabled
- ROS 2 Humble Hawksbill installed
- Basic understanding of ROS 2 concepts (topics, messages, nodes)

## Learning Objectives

By the end of this lab, you will be able to:
- Set up Isaac ROS perception packages
- Create a hardware-accelerated perception pipeline
- Integrate different perception algorithms
- Process sensor data in real-time
- Visualize perception results in Isaac Sim

## Background

Isaac ROS perception packages provide GPU-accelerated computer vision algorithms that run on NVIDIA hardware. These packages include:

- **DetectNet**: Object detection with bounding boxes
- **PoseNet**: 6D pose estimation for objects
- **Segmentation**: Semantic and instance segmentation
- **Depth estimation**: Stereo vision and monocular depth estimation
- **Image pipeline**: Hardware-accelerated image processing

## Setting Up Isaac ROS Perception

### Step 1: Verify Isaac ROS Installation

First, verify that Isaac ROS packages are available in your ROS 2 environment:

```bash
# Source ROS 2 and Isaac ROS packages
source /opt/ros/humble/setup.bash
# If Isaac ROS is installed as a ROS package
source /usr/local/share/isaac_ros_common/setup.bash

# Check available Isaac ROS packages
ros2 pkg list | grep isaac
```

### Step 2: Install Isaac ROS Dependencies

Install the necessary Isaac ROS packages:

```bash
# Install Isaac ROS perception packages
sudo apt update
sudo apt install -y ros-humble-isaac-ros-perception
sudo apt install -y ros-humble-isaac-ros-apriltag
sudo apt install -y ros-humble-isaac-ros-visual-slam
sudo apt install -y ros-humble-isaac-ros-message-filters
sudo apt install -y ros-humble-isaac-ros-gxf
```

### Step 3: Verify GPU Acceleration

Check that CUDA and TensorRT are properly configured:

```bash
# Verify CUDA
nvidia-smi
nvcc --version

# Test TensorRT
python3 -c "import tensorrt as trt; print('TensorRT version:', trt.__version__)"
```

## Creating a Basic Perception Pipeline

### Step 4: Create a ROS 2 Workspace

Create a workspace for your perception pipeline:

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Create a package for the perception pipeline
cd src
ros2 pkg create --build-type ament_python perception_pipeline
cd perception_pipeline
```

### Step 5: Build a Simple Object Detection Pipeline

Create a basic object detection pipeline using Isaac ROS DetectNet. First, create the main launch file:

Create `~/isaac_ros_ws/src/perception_pipeline/launch/detection_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch arguments
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    input_topic_name = LaunchConfiguration('input_topic_name')
    output_topic_name = LaunchConfiguration('output_topic_name')

    model_file_path_arg = DeclareLaunchArgument(
        'model_file_path',
        default_value='/usr/share/vision_msgs/detectnet/models/ssd_mobilenet_v2_coco_2018_03_29.onnx',
        description='Path to the model file'
    )

    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value='/tmp/detectnet_mobilenet.engine',
        description='Path to the TensorRT engine file'
    )

    input_topic_name_arg = DeclareLaunchArgument(
        'input_topic_name',
        default_value='/camera/rgb/image_raw',
        description='Input topic name'
    )

    output_topic_name_arg = DeclareLaunchArgument(
        'output_topic_name',
        default_value='/detectnet/detections',
        description='Output topic name'
    )

    # Create container for the nodes
    detectnet_container = ComposableNodeContainer(
        name='detectnet_container',
        namespace='detectnet',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::dnn_image_encoder::DnnImageEncoderNode',
                name='dnn_encoder',
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'network_image_width': 300,
                    'network_image_height': 300,
                    'network_scale_factor': 0.007843,
                    'network_mean': [127.5, 127.5, 127.5],
                    'network_stddev': [1.0, 1.0, 1.0],
                    'image_mean': [103.939, 116.779, 123.68],
                    'image_stddev': [1.0, 1.0, 1.0],
                    'network_input_layer_name': 'image',
                    'tensor_layout': 'NHWC',
                    'tensor_name': 'input',
                    'image_input_layer_name': 'input',
                    'input_tensor_rows': 300,
                    'input_tensor_cols': 300,
                    'input_tensor_channels': 3,
                    'layer_names': ['image'],
                    'dtype': 'uint8',
                    'batch_size': 1,
                    'do_dynamic_batching': False,
                }],
                remappings=[
                    ('encoded_tensor', 'tensor_pub'),
                    ('image', input_topic_name),
                ]
            ),
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='detectnet',
                parameters=[{
                    'model_file_path': model_file_path,
                    'engine_file_path': engine_file_path,
                    'input_tensor_name': 'input',
                    'input_resized_tensor_name': 'downsampled_input',
                    'output_coverage_name': 'scores',
                    'output_bbox_name': 'boxes',
                    'output_categ_names': ['classes'],
                    'thresh': 0.7,
                    'top_k': 100,
                    'enable_padding': True,
                    'max_batch_size': 1,
                }],
                remappings=[
                    ('detections', output_topic_name),
                    ('tensor_sub', 'tensor_pub'),
                ]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        model_file_path_arg,
        engine_file_path_arg,
        input_topic_name_arg,
        output_topic_name_arg,
        detectnet_container,
    ])
```

### Step 6: Create a Simple Perception Node

Create a basic perception node that processes sensor data:

Create `~/isaac_ros_ws/src/perception_pipeline/perception_pipeline/perception_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # Create publishers
        self.result_pub = self.create_publisher(Image, '/perception/result', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.get_logger().info('Perception Node has been started')

    def image_callback(self, msg):
        """Process incoming image and visualize detections"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # In a real implementation, you would combine the image with detection results
            # For now, just publish the original image
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detection_callback(self, msg):
        """Process detection results"""
        self.get_logger().info(f'Received {len(msg.detections)} detections')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Create Package Configuration

Create the package configuration file `~/isaac_ros_ws/src/perception_pipeline/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'perception_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detection_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Isaac ROS Perception Pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = perception_pipeline.perception_node:main',
        ],
    },
)
```

Create the package manifest `~/isaac_ros_ws/src/perception_pipeline/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception_pipeline</name>
  <version>0.0.1</version>
  <description>Isaac ROS Perception Pipeline</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>

  <exec_depend>isaac_ros_detectnet</exec_depend>
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

## Building and Testing the Pipeline

### Step 8: Build the Workspace

```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_pipeline
source install/setup.bash
```

### Step 9: Run the Perception Pipeline

To test the pipeline, you'll need to simulate camera data. You can use Isaac Sim to generate camera data or use a pre-recorded dataset:

```bash
# Terminal 1: Launch the perception pipeline
source ~/isaac_ros_ws/install/setup.bash
ros2 launch perception_pipeline detection_pipeline.launch.py

# Terminal 2: Run the perception node
source ~/isaac_ros_ws/install/setup.bash
ros2 run perception_pipeline perception_node
```

## Advanced Perception Pipeline Components

### Step 10: Adding Stereo Depth Estimation

To enhance the perception pipeline, you can add stereo depth estimation:

```python
# Add this to your launch file for stereo processing
ComposableNode(
    package='isaac_ros_stereo_image_proc',
    plugin='nvidia::isaac_ros::stereo_image_proc::SgmNode',
    name='sgm_node',
    parameters=[{
        'min_disparity': 0,
        'max_disparity': 64,
        'num_disparities': 64,
        'disp12_max_diff': 1,
        'uniqueness_ratio': 10,
        'speckle_window_size': 0,
        'speckle_range': 0,
        'p1': 200,
        'p2': 400,
    }],
    remappings=[
        ('left/image_rect', '/camera/left/image_rect_color'),
        ('right/image_rect', '/camera/right/image_rect_color'),
        ('left/camera_info', '/camera/left/camera_info'),
        ('right/camera_info', '/camera/right/camera_info'),
        ('disparity', '/disparity'),
    ]
)
```

### Step 11: Adding Pose Estimation

Add 6D pose estimation to detect and estimate the pose of objects:

```python
# Add this to your launch file for pose estimation
ComposableNode(
    package='isaac_ros_pose_estimation',
    plugin='nvidia::isaac_ros::pose_estimation::PoseEstimationNode',
    name='pose_estimation_node',
    parameters=[{
        'model_file_path': '/path/to/pose_model.onnx',
        'engine_file_path': '/tmp/pose_estimation.engine',
        'num_objects': 1,
        'confidence_threshold': 0.7,
        'class_labels': ['object1', 'object2'],
    }],
    remappings=[
        ('input_image', '/camera/rgb/image_raw'),
        ('output_poses', '/pose_estimation/poses'),
    ]
)
```

## Integration with Isaac Sim

### Step 12: Connect Isaac Sim to Your Pipeline

To connect Isaac Sim to your perception pipeline:

1. In Isaac Sim, create a scene with a robot that has RGB and depth cameras
2. Configure the cameras to publish to ROS topics:
   - RGB camera: `/camera/rgb/image_raw`
   - Depth camera: `/camera/depth/image_raw`
   - Camera info: `/camera/rgb/camera_info`

3. The Isaac ROS perception pipeline will automatically process the sensor data from Isaac Sim

## Troubleshooting Common Issues

### Issue 1: CUDA/TensorRT Compatibility
**Symptoms**: Perception nodes fail to initialize
**Solution**:
- Verify CUDA and TensorRT versions are compatible
- Check that Isaac ROS packages were built with the correct CUDA version
- Ensure GPU supports the required compute capability

### Issue 2: Performance Issues
**Symptoms**: Low frame rate, high latency
**Solution**:
- Optimize network input size
- Reduce batch size if using multiple models
- Check GPU memory usage

### Issue 3: Model Loading Failures
**Symptoms**: "Failed to load model" errors
**Solution**:
- Verify model file paths are correct
- Check that model files exist and are readable
- Ensure model format is supported (ONNX, TensorRT engine)

## Verification

To verify that your perception pipeline is working correctly:

1. Perception nodes launch without errors
2. Input images are processed successfully
3. Detection results are published to appropriate topics
4. Frame rate is acceptable for real-time operation
5. GPU utilization is visible when processing images

## Performance Optimization Tips

1. **Model Optimization**: Use TensorRT to optimize neural networks
2. **Batch Processing**: Process multiple images in batches when possible
3. **Memory Management**: Use CUDA unified memory for efficient data transfer
4. **Pipeline Parallelization**: Process different aspects of perception in parallel

## Summary

In this lab, you've successfully built a hardware-accelerated perception pipeline using Isaac ROS. You've learned how to create a detection pipeline, integrate different perception components, and connect the pipeline to Isaac Sim. This forms the foundation for more advanced perception tasks in robotics applications.

## Next Steps

In Lab 3, you'll implement Isaac ROS Visual SLAM to enable autonomous navigation and mapping capabilities for your robotic system.