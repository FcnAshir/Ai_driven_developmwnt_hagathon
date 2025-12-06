# Lab 2: Build a Perception Pipeline

## Objective
Implement a basic perception pipeline using Isaac ROS components.

## Prerequisites
- Completed Lab 1 (Isaac Sim installation)
- ROS 2 Humble installed
- Isaac ROS packages installed
- Basic understanding of ROS 2 concepts

## Steps

### 1. Create a perception pipeline package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python perception_pipeline_pkg
cd perception_pipeline_pkg
```

### 2. Install Isaac ROS perception packages
```bash
sudo apt update
sudo apt install ros-humble-isaac-ros-point-cloud-transport ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-apriltag ros-humble-isaac-ros-visual-slam
```

### 3. Create a perception pipeline node
Create `perception_pipeline_pkg/perception_pipeline.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # QoS profile for synchronized subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for camera data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos_profile
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/color/camera_info', qos_profile=qos_profile
        )

        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.camera_callback)

        # Create publishers for processed data
        self.object_detection_pub = self.create_publisher(
            Detection2DArray, '/perception/object_detections', 10
        )
        self.processed_image_pub = self.create_publisher(
            Image, '/perception/processed_image', 10
        )
        self.point_cloud_pub = self.create_publisher(
            PointCloud2, '/perception/point_cloud', 10
        )

        # Perception parameters
        self.detection_threshold = 0.5
        self.processing_rate = 10  # Hz
        self.timer = self.create_timer(1.0/self.processing_rate, self.process_timer)

        # Processing buffers
        self.latest_image = None
        self.latest_camera_info = None
        self.processing_counter = 0

        self.get_logger().info('Perception Pipeline Node Started')

    def camera_callback(self, image_msg, camera_info_msg):
        """Receive synchronized image and camera info."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Store for processing
            self.latest_image = cv_image
            self.latest_camera_info = camera_info_msg

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def process_timer(self):
        """Process perception pipeline at regular intervals."""
        if self.latest_image is not None:
            # Process the image through perception pipeline
            processed_image, detections = self.process_perception_pipeline(
                self.latest_image, self.latest_camera_info
            )

            # Publish results
            self.publish_detections(detections)
            self.publish_processed_image(processed_image)

            # Increment counter for statistics
            self.processing_counter += 1

            # Log periodically
            if self.processing_counter % (self.processing_rate * 5) == 0:  # Every 5 seconds
                self.get_logger().info(
                    f'Perception pipeline processed {self.processing_counter} frames'
                )

    def process_perception_pipeline(self, image, camera_info):
        """Main perception pipeline processing function."""
        # Create a copy of the image for processing
        processed_image = image.copy()

        # 1. Object Detection using color-based segmentation
        detections = self.detect_color_objects(processed_image)

        # 2. Feature Detection
        features = self.detect_features(processed_image)

        # 3. Draw detections on image
        for detection in detections:
            # Draw bounding box
            x, y, w, h = detection['bbox']
            cv2.rectangle(processed_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw label
            label = f"{detection['label']}: {detection['confidence']:.2f}"
            cv2.putText(processed_image, label, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return processed_image, detections

    def detect_color_objects(self, image):
        """Detect objects based on color ranges."""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges (HSV format)
        color_ranges = {
            'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
            'yellow': (np.array([20, 50, 50]), np.array([30, 255, 255]))
        }

        detections = []

        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for color range
            mask = cv2.inRange(hsv, lower, upper)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Filter by area
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    detection = {
                        'label': color_name,
                        'confidence': min(0.9, area / 10000),  # Normalize confidence
                        'bbox': (x, y, w, h),
                        'area': area
                    }
                    detections.append(detection)

        return detections

    def detect_features(self, image):
        """Detect features using traditional computer vision."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use ORB for feature detection
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Draw keypoints on image
        output_image = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0))

        return {
            'keypoints': keypoints,
            'descriptors': descriptors,
            'count': len(keypoints) if keypoints is not None else 0
        }

    def publish_detections(self, detections):
        """Publish object detections."""
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_color_optical_frame'

        # Convert detections to vision_msgs format
        for detection in detections:
            if detection['confidence'] > self.detection_threshold:
                # Create a detection object
                vision_detection = Detection2D()
                vision_detection.header = detection_array.header
                vision_detection.results = []  # We'll add classification results if needed

                # Set bounding box
                vision_detection.bbox.center.x = detection['bbox'][0] + detection['bbox'][2] / 2
                vision_detection.bbox.center.y = detection['bbox'][1] + detection['bbox'][3] / 2
                vision_detection.bbox.size_x = detection['bbox'][2]
                vision_detection.bbox.size_y = detection['bbox'][3]

                detection_array.detections.append(vision_detection)

        self.object_detection_pub.publish(detection_array)

    def publish_processed_image(self, processed_image):
        """Publish the processed image."""
        try:
            # Convert OpenCV image back to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_color_optical_frame'

            self.processed_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    perception_pipeline = PerceptionPipeline()

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Create a more advanced perception pipeline using Isaac ROS components
Create `perception_pipeline_pkg/isaac_perception_pipeline.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class IsaacPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('isaac_perception_pipeline')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # QoS profile for synchronized subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for Isaac Sim camera data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/isaac_sim_camera/image', qos_profile=qos_profile
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/isaac_sim_camera/camera_info', qos_profile=qos_profile
        )

        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.camera_callback)

        # Create publishers for processed data
        self.object_detection_pub = self.create_publisher(
            Detection2DArray, '/isaac_perception/object_detections', 10
        )
        self.segmentation_pub = self.create_publisher(
            Image, '/isaac_perception/segmentation', 10
        )
        self.depth_pub = self.create_publisher(
            Image, '/isaac_perception/depth', 10
        )

        # Perception parameters
        self.processing_rate = 15  # Hz (Isaac Sim typically runs at 60Hz)
        self.timer = self.create_timer(1.0/self.processing_rate, self.process_timer)

        # Processing buffers
        self.latest_image = None
        self.latest_camera_info = None
        self.processing_counter = 0

        # Initialize perception components
        self.setup_perception_components()

        self.get_logger().info('Isaac Perception Pipeline Node Started')

    def setup_perception_components(self):
        """Initialize perception components."""
        # For this example, we'll use OpenCV-based components
        # In a real Isaac ROS setup, you'd use Isaac ROS extensions

        # Initialize HOG descriptor for person detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def camera_callback(self, image_msg, camera_info_msg):
        """Receive synchronized image and camera info from Isaac Sim."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Store for processing
            self.latest_image = cv_image
            self.latest_camera_info = camera_info_msg

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def process_timer(self):
        """Process perception pipeline at regular intervals."""
        if self.latest_image is not None:
            # Process the image through Isaac-style perception pipeline
            segmentation_img, detections = self.process_isaac_perception_pipeline(
                self.latest_image, self.latest_camera_info
            )

            # Publish results
            self.publish_detections(detections)
            self.publish_segmentation(segmentation_img)

            # Increment counter for statistics
            self.processing_counter += 1

            # Log periodically
            if self.processing_counter % (self.processing_rate * 5) == 0:  # Every 5 seconds
                self.get_logger().info(
                    f'Isaac perception pipeline processed {self.processing_counter} frames'
                )

    def process_isaac_perception_pipeline(self, image, camera_info):
        """Isaac-style perception pipeline processing function."""
        # Create a copy of the image for processing
        processed_image = image.copy()
        height, width = image.shape[:2]

        # 1. Object Detection using HOG (for person detection)
        detections = self.detect_people_with_hog(processed_image)

        # 2. Semantic Segmentation (simulated)
        segmentation_mask = self.create_semantic_segmentation(processed_image)

        # 3. Feature Extraction
        features = self.extract_features(processed_image)

        return segmentation_mask, detections

    def detect_people_with_hog(self, image):
        """Detect people using HOG descriptor."""
        # Resize image for better detection
        resized_img = cv2.resize(image, (640, 480))

        # Detect people
        boxes, weights = self.hog.detectMultiScale(resized_img, winStride=(8, 8))

        detections = []
        for (x, y, w, h), weight in zip(boxes, weights):
            if weight > 0.5:  # Confidence threshold
                # Scale back to original size
                scale_x = image.shape[1] / 640.0
                scale_y = image.shape[0] / 480.0

                detection = {
                    'label': 'person',
                    'confidence': min(0.95, float(weight)),
                    'bbox': (int(x * scale_x), int(y * scale_y),
                            int(w * scale_x), int(h * scale_y)),
                    'center': (int((x + w/2) * scale_x), int((y + h/2) * scale_y))
                }
                detections.append(detection)

        return detections

    def create_semantic_segmentation(self, image):
        """Create a simulated semantic segmentation mask."""
        # In a real implementation, this would use a deep learning model
        # For simulation, we'll create a simple color-based segmentation

        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks for different color ranges
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine masks
        combined_mask = np.zeros_like(mask_red)
        combined_mask[mask_red > 0] = 50   # Red objects
        combined_mask[mask_green > 0] = 100 # Green objects
        combined_mask[mask_blue > 0] = 150 # Blue objects

        # Convert to 3-channel for ROS publishing
        segmentation_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

        return segmentation_bgr

    def extract_features(self, image):
        """Extract features using various methods."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use SIFT for feature detection
        sift = cv2.SIFT_create()
        keypoints, descriptors = sift.detectAndCompute(gray, None)

        return {
            'keypoints': keypoints,
            'descriptors': descriptors,
            'count': len(keypoints) if keypoints is not None else 0
        }

    def publish_detections(self, detections):
        """Publish object detections in Isaac-compatible format."""
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'isaac_sim_camera_optical_frame'

        # Convert detections to vision_msgs format
        for detection in detections:
            if detection['confidence'] > 0.5:  # Threshold
                # Create a detection object
                vision_detection = Detection2D()
                vision_detection.header = detection_array.header
                vision_detection.results = []  # Isaac ROS would populate this

                # Set bounding box
                bbox_x, bbox_y, bbox_w, bbox_h = detection['bbox']
                vision_detection.bbox.center.x = bbox_x + bbox_w / 2
                vision_detection.bbox.center.y = bbox_y + bbox_h / 2
                vision_detection.bbox.size_x = bbox_w
                vision_detection.bbox.size_y = bbox_h

                detection_array.detections.append(vision_detection)

        self.object_detection_pub.publish(detection_array)

    def publish_segmentation(self, segmentation_image):
        """Publish segmentation mask."""
        try:
            # Convert OpenCV image back to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(segmentation_image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'isaac_sim_camera_optical_frame'

            self.segmentation_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing segmentation: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    isaac_perception_pipeline = IsaacPerceptionPipeline()

    try:
        rclpy.spin(isaac_perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5. Create a launch file for the perception pipeline
Create `launch/perception_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate',
        default_value='10',
        description='Processing rate for perception pipeline'
    )

    processing_rate = LaunchConfiguration('processing_rate')

    return LaunchDescription([
        processing_rate_arg,

        # Basic perception pipeline
        Node(
            package='perception_pipeline_pkg',
            executable='perception_pipeline',
            name='perception_pipeline',
            output='screen',
            parameters=[
                {'processing_rate': processing_rate}
            ]
        ),

        # Isaac-style perception pipeline
        Node(
            package='perception_pipeline_pkg',
            executable='isaac_perception_pipeline',
            name='isaac_perception_pipeline',
            output='screen',
            parameters=[
                {'processing_rate': processing_rate}
            ]
        )
    ])
```

### 6. Update setup.py for the perception package
Edit `setup.py`:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'perception_pipeline_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Perception pipeline for Isaac Sim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_pipeline = perception_pipeline_pkg.perception_pipeline:main',
            'isaac_perception_pipeline = perception_pipeline_pkg.isaac_perception_pipeline:main',
        ],
    },
)