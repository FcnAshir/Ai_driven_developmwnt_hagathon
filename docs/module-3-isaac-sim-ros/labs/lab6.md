# Lab 6: Deploy ROS Nodes to Jetson Orin

## Objective
Deploy and run ROS 2 applications on NVIDIA Jetson Orin embedded platforms.

## Prerequisites
- NVIDIA Jetson Orin development kit
- JetPack SDK installed (5.1 or later)
- ROS 2 Humble Hawksbill
- Cross-compilation environment (if applicable)
- Understanding of embedded systems constraints

## Steps

### 1. Prepare Jetson Orin Environment
First, ensure your Jetson Orin is properly set up:

```bash
# Update the system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble dependencies
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install ROS 2 Humble
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-nav2-bringup
```

### 2. Install Jetson-specific packages and optimizations
```bash
# Install Jetson Inference (for AI workloads)
sudo apt install libjetson-inference-dev libjetson-utils-dev

# Install CUDA and TensorRT (pre-installed with JetPack, but verify)
sudo apt install cuda-toolkit-11-4 libnvinfer8 libnvinfer-dev libnvparsers8 libnvonnxparsers8

# Install performance monitoring tools
sudo apt install jp-toolkits
```

### 3. Set up ROS workspace on Jetson
```bash
# Create workspace
mkdir -p ~/ros2_jetson_ws/src
cd ~/ros2_jetson_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace (initially empty)
colcon build --symlink-install
source install/setup.bash
```

### 4. Create a Jetson-optimized perception node
Create `~/ros2_jetson_ws/src/jetson_perception/jetson_perception_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters


class JetsonPerceptionNode(Node):
    def __init__(self):
        super().__init__('jetson_perception_node')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # QoS profile optimized for embedded systems
        qos_profile = QoSProfessional(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for camera data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/image_raw', qos_profile=qos_profile
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera_info', qos_profile=qos_profile
        )

        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], queue_size=1, slop=0.1
        )
        self.sync.registerCallback(self.camera_callback)

        # Publishers for results
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/jetson/detections', 10
        )
        self.processed_image_pub = self.create_publisher(
            Image, '/jetson/processed_image', 10
        )

        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        # Processing parameters optimized for Jetson
        self.processing_rate = 15  # Lower rate for power efficiency
        self.timer = self.create_timer(1.0/self.processing_rate, self.process_timer)

        # Buffers for processing
        self.latest_image = None
        self.latest_camera_info = None
        self.processing_lock = threading.Lock()

        # Jetson-specific optimizations
        self.enable_gpu_processing = True
        self.use_tensorrt = False  # Set to True if TensorRT models are available

        # Initialize detectors (optimized for Jetson)
        self.detector = self.initialize_jetson_detector()

        self.get_logger().info('Jetson Perception Node Started - Optimized for Orin')

    def initialize_jetson_detector(self):
        """Initialize detectors optimized for Jetson Orin."""
        # For this example, we'll use OpenCV-based detection
        # In practice, you'd use TensorRT-optimized models

        # Use HOG for people detection (less resource intensive)
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        return hog

    def camera_callback(self, image_msg, camera_info_msg):
        """Receive camera data from sensor."""
        try:
            # Convert ROS Image to OpenCV with Jetson optimizations
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Store for processing (with thread safety)
            with self.processing_lock:
                self.latest_image = cv_image.copy()  # Copy to avoid reference issues
                self.latest_camera_info = camera_info_msg

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def process_timer(self):
        """Process perception pipeline at regular intervals."""
        # Get latest image (with thread safety)
        with self.processing_lock:
            if self.latest_image is None:
                return

            image = self.latest_image.copy()
            camera_info = self.latest_camera_info

        # Process the image
        processed_image, detections = self.process_perception_pipeline(image, camera_info)

        # Publish results
        self.publish_detections(detections, image_msg.header if hasattr(image_msg, 'header') else None)
        self.publish_processed_image(processed_image)

        # Update performance metrics
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed >= 1.0:  # Update FPS every second
            self.fps = self.frame_count / elapsed
            self.get_logger().info(f'Jetson Perception FPS: {self.fps:.2f}')
            self.frame_count = 0
            self.start_time = time.time()

    def process_perception_pipeline(self, image, camera_info):
        """Jetson-optimized perception pipeline."""
        height, width = image.shape[:2]

        # Resize image for faster processing on Jetson
        target_width, target_height = 320, 240  # Reduced resolution
        if width > target_width:
            scale_factor = target_width / width
            new_width = int(width * scale_factor)
            new_height = int(height * scale_factor)
            image = cv2.resize(image, (new_width, new_height))

        # Perform detection using HOG (optimized for CPU on Jetson)
        detections = self.detect_people_hog(image)

        # Draw detections on image
        processed_image = image.copy()
        for detection in detections:
            bbox = detection['bbox']
            cv2.rectangle(processed_image, (bbox[0], bbox[1]),
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)
            cv2.putText(processed_image, f"Person {detection['confidence']:.2f}",
                       (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return processed_image, detections

    def detect_people_hog(self, image):
        """Detect people using HOG descriptor (CPU-optimized)."""
        # Convert to appropriate size
        if image.shape[0] < 64 or image.shape[1] < 64:  # Minimum HOG size
            return []

        # Detect people using HOG
        boxes, weights = self.detector.detectMultiScale(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
            winStride=(8, 8),
            padding=(16, 16),
            scale=1.05
        )

        detections = []
        for (x, y, w, h), weight in zip(boxes, weights):
            if weight > 0.5:  # Confidence threshold
                detection = {
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'confidence': min(0.95, float(weight)),
                    'label': 'person'
                }
                detections.append(detection)

        return detections

    def publish_detections(self, detections, header=None):
        """Publish detection results."""
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_link'

        for detection in detections:
            if detection['confidence'] > 0.5:
                det_msg = Detection2D()
                det_msg.header = detection_array.header

                # Set bounding box
                bbox_x, bbox_y, bbox_w, bbox_h = detection['bbox']
                det_msg.bbox.center.x = bbox_x + bbox_w / 2
                det_msg.bbox.center.y = bbox_y + bbox_h / 2
                det_msg.bbox.size_x = bbox_w
                det_msg.bbox.size_y = bbox_h

                detection_array.detections.append(det_msg)

        self.detection_pub.publish(detection_array)

    def publish_processed_image(self, processed_image):
        """Publish the processed image."""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'

            self.processed_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {str(e)}')

    def get_jetson_performance_stats(self):
        """Get performance statistics from Jetson platform."""
        # This is a simplified version - in practice you'd read from Jetson tools
        try:
            # Get CPU usage (simplified)
            import psutil
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent

            self.get_logger().info(
                f'Jetson Performance - CPU: {cpu_percent}%, Memory: {memory_percent}%'
            )
        except ImportError:
            self.get_logger().info('psutil not available for performance monitoring')


def main(args=None):
    rclpy.init(args=args)
    jetson_perception_node = JetsonPerceptionNode()

    try:
        rclpy.spin(jetson_perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        jetson_perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5. Create a power management and optimization node
Create `~/ros2_jetson_ws/src/jetson_utils/power_manager.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import subprocess
import time
import threading
import os


class JetsonPowerManager(Node):
    def __init__(self):
        super().__init__('jetson_power_manager')

        # Publishers for system metrics
        self.cpu_usage_pub = self.create_publisher(Float32, 'jetson/cpu_usage', 10)
        self.gpu_usage_pub = self.create_publisher(Float32, 'jetson/gpu_usage', 10)
        self.temperature_pub = self.create_publisher(Float32, 'jetson/temperature', 10)
        self.power_mode_pub = self.create_publisher(String, 'jetson/power_mode', 10)

        # Parameters
        self.monitoring_rate = 2.0  # Hz
        self.power_threshold = 80.0  # Percent
        self.temperature_threshold = 80.0  # Celsius

        # Start monitoring
        self.monitor_timer = self.create_timer(1.0/self.monitoring_rate, self.monitor_system)

        # Power mode management
        self.current_power_mode = 'MAXN'  # MAXN, MODE_15W, MODE_10W
        self.thermal_protective_mode = False

        self.get_logger().info('Jetson Power Manager Started')

    def monitor_system(self):
        """Monitor system resources and adjust accordingly."""
        # Get CPU usage
        cpu_usage = self.get_cpu_usage()
        cpu_msg = Float32()
        cpu_msg.data = cpu_usage
        self.cpu_usage_pub.publish(cpu_msg)

        # Get GPU usage (simplified)
        gpu_usage = self.get_gpu_usage()
        gpu_msg = Float32()
        gpu_msg.data = gpu_usage
        self.gpu_usage_pub.publish(gpu_msg)

        # Get temperature
        temperature = self.get_temperature()
        temp_msg = Float32()
        temp_msg.data = temperature
        self.temperature_pub.publish(temp_msg)

        # Check thresholds and adjust power mode
        self.check_power_thresholds(cpu_usage, temperature)

        # Log status periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:  # Every 10 seconds
            self.get_logger().info(
                f'Power Status - CPU: {cpu_usage:.1f}%, GPU: {gpu_usage:.1f}%, '
                f'Temp: {temperature:.1f}C, Mode: {self.current_power_mode}'
            )

    def get_cpu_usage(self):
        """Get CPU usage percentage."""
        try:
            # Simple CPU usage check
            import psutil
            return psutil.cpu_percent()
        except ImportError:
            # Fallback to basic system command
            result = subprocess.run(['top', '-bn1'], capture_output=True, text=True)
            # This is a simplified implementation
            return 50.0  # Placeholder

    def get_gpu_usage(self):
        """Get GPU usage percentage."""
        try:
            # Use nvidia-smi to get GPU usage
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                capture_output=True, text=True
            )
            if result.stdout.strip():
                return float(result.stdout.strip())
        except Exception:
            pass
        return 0.0

    def get_temperature(self):
        """Get system temperature."""
        try:
            # Read temperature from Jetson thermal zone
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000.0  # Convert from millidegrees
                return temp
        except Exception:
            return 35.0  # Default temperature

    def check_power_thresholds(self, cpu_usage, temperature):
        """Check if system parameters exceed thresholds and adjust accordingly."""
        high_load = cpu_usage > self.power_threshold
        high_temp = temperature > self.temperature_threshold

        if high_temp and not self.thermal_protective_mode:
            self.thermal_protective_mode = True
            self.get_logger().warn(f'High temperature detected: {temperature:.1f}C, reducing power mode')
            self.set_power_mode('MODE_10W')
        elif not high_temp and self.thermal_protective_mode:
            self.thermal_protective_mode = False
            self.get_logger().info('Temperature normal, restoring power mode')
            self.set_power_mode('MAXN')

    def set_power_mode(self, mode):
        """Set Jetson power mode."""
        try:
            if mode == 'MAXN':
                # Set to maximum performance mode
                subprocess.run(['sudo', 'nvpmodel', '-m', '0'], check=True)
                # Set fan to automatic
                subprocess.run(['sudo', 'pwmconfig'], input='1\n', text=True, timeout=5)
            elif mode == 'MODE_15W':
                subprocess.run(['sudo', 'nvpmodel', '-m', '1'], check=True)
            elif mode == 'MODE_10W':
                subprocess.run(['sudo', 'nvpmodel', '-m', '2'], check=True)

            self.current_power_mode = mode
            mode_msg = String()
            mode_msg.data = mode
            self.power_mode_pub.publish(mode_msg)

            self.get_logger().info(f'Power mode set to: {mode}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to set power mode {mode}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error setting power mode: {e}')


def main(args=None):
    rclpy.init(args=args)
    power_manager = JetsonPowerManager()

    try:
        rclpy.spin(power_manager)
    except KeyboardInterrupt:
        pass
    finally:
        power_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6. Create a deployment configuration file
Create `~/ros2_jetson_ws/src/jetson_deployment/config/jetson_deployment.yaml`:

```yaml
# Jetson Orin Deployment Configuration

/**:
  ros__parameters:
    # Performance parameters
    jetson_performance_mode: "MAXN"  # MAXN, MODE_15W, MODE_10W
    enable_gpu_processing: true
    use_tensorrt: false
    processing_rate: 15.0  # Hz

    # Resource constraints
    max_cpu_usage: 80.0    # Percentage
    max_gpu_usage: 85.0    # Percentage
    max_temperature: 80.0  # Celsius

    # Memory management
    enable_memory_monitoring: true
    low_memory_threshold: 80.0  # Percentage

    # Communication settings for embedded systems
    qos_reliability: "best_effort"
    qos_history: "keep_last"
    qos_depth: 1

    # Power management
    enable_power_management: true
    power_management_rate: 2.0  # Hz

jetson_perception_node:
  ros__parameters:
    # Perception-specific settings
    image_processing_resolution: [320, 240]  # Reduced for performance
    detection_confidence_threshold: 0.5
    max_detections_per_frame: 10
    enable_object_tracking: false  # Disable for performance

    # Sensor parameters
    camera_frame_id: "camera_link"
    camera_info_topic: "/camera/camera_info"
    image_topic: "/camera/image_raw"

jetson_power_manager:
  ros__parameters:
    # Power management settings
    cpu_threshold: 80.0
    temperature_threshold: 80.0
    monitoring_rate: 2.0
    thermal_protection_enabled: true
```

### 7. Create a launch file for Jetson deployment
Create `~/ros2_jetson_ws/src/jetson_deployment/launch/jetson_deployment.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jetson_deployment'),
            'config',
            'jetson_deployment.yaml'
        ]),
        description='Path to Jetson deployment configuration file'
    )

    # Set environment variables for Jetson optimization
    set_env_vars = [
        SetEnvironmentVariable(name='CUDA_VISIBLE_DEVICES', value='0'),
        SetEnvironmentVariable(name='PYTHONPATH', value='/opt/ros/humble/lib/python3.8/site-packages'),
    ]

    # Jetson Perception Node
    jetson_perception_node = Node(
        package='jetson_perception',
        executable='jetson_perception_node',
        name='jetson_perception_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ],
        # Set CPU affinity for better performance
        on_exit=None  # Keep running
    )

    # Jetson Power Manager
    jetson_power_manager = Node(
        package='jetson_utils',
        executable='power_manager',
        name='jetson_power_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )

    # Optional: RViz2 for debugging (only if display is available)
    # Comment out if running headless
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='jetson_rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('jetson_deployment'),
            'rviz',
            'jetson_deployment.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
    ] + set_env_vars + [
        jetson_perception_node,
        jetson_power_manager,
        # rviz_node  # Uncomment if display is available
    ])
```

### 8. Create startup script for Jetson
Create `~/ros2_jetson_ws/src/jetson_deployment/scripts/startup_jetson.sh`:

```bash
#!/bin/bash

# Jetson Orin ROS 2 Startup Script

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
cd ~/ros2_jetson_ws
source install/setup.bash

# Set Jetson performance mode (MAXN for full performance)
sudo nvpmodel -m 0
sudo jetson_clocks

# Set environment variables for optimal performance
export CUDA_VISIBLE_DEVICES=0
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Optional: Set CPU governor to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Create log directory
mkdir -p ~/jetson_logs
LOG_FILE=~/jetson_logs/$(date +%Y%m%d_%H%M%S)_jetson.log

echo "Starting Jetson ROS 2 application..." | tee -a $LOG_FILE

# Launch the application
ros2 launch jetson_deployment jetson_deployment.launch.py 2>&1 | tee -a $LOG_FILE

# On exit, restore default power mode
echo "Shutting down - restoring default power mode"
sudo nvpmodel -m 0
```

### 9. Make the startup script executable
```bash
chmod +x ~/ros2_jetson_ws/src/jetson_deployment/scripts/startup_jetson.sh
```

### 10. Build and deploy
```bash
cd ~/ros2_jetson_ws

# Build the workspace
colcon build --packages-select jetson_perception jetson_utils jetson_deployment

# Source the workspace
source install/setup.bash

# Run the application
ros2 launch jetson_deployment jetson_deployment.launch.py
```

### 11. Set up auto-start on boot (optional)
Create a systemd service file on Jetson to auto-start your ROS 2 application:

```bash
sudo nano /etc/systemd/system/jetson-ros2.service
```

Add the following content:

```ini
[Unit]
Description=Jetson ROS 2 Service
After=multi-user.target

[Service]
Type=simple
User=jetson
Group=jetson
ExecStart=/home/jetson/ros2_jetson_ws/src/jetson_deployment/scripts/startup_jetson.sh
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
Environment="ROS_DOMAIN_ID=0"

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable jetson-ros2.service
sudo systemctl start jetson-ros2.service
```

## Expected Output
- ROS 2 nodes running efficiently on Jetson Orin
- Optimized resource usage (CPU, GPU, memory, power)
- Temperature and performance monitoring
- Perception pipeline running at target frame rate
- Proper QoS settings for embedded communication

## Troubleshooting
- If nodes don't start, check ROS domain settings
- If performance is poor, verify power mode settings
- If memory usage is high, reduce image processing resolution
- If thermal issues occur, adjust power mode or cooling

## Next Steps
- Implement TensorRT acceleration for AI models
- Add more sophisticated power management
- Integrate with real sensors on Jetson carrier board
- Optimize for specific robotic applications