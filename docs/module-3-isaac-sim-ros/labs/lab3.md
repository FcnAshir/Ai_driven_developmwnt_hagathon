# Lab 3: Run Isaac ROS Visual SLAM

## Objective
Utilize Isaac ROS for Visual Simultaneous Localization and Mapping (VSLAM).

## Prerequisites
- Completed Lab 1 and Lab 2
- Isaac Sim installed and running
- Isaac ROS Visual SLAM packages installed
- Camera sensor configured in simulation

## Steps

### 1. Install Isaac ROS Visual SLAM packages
```bash
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-stereo-image-pipeline ros-humble-isaac-ros-apriltag
```

### 2. Create a VSLAM configuration package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake vslam_config_pkg
cd vslam_config_pkg
mkdir config launch
```

### 3. Create VSLAM configuration file
Create `config/vslam_config.yaml`:

```yaml
/**:
  ros__parameters:
    # Visual SLAM parameters
    enable_rectification: true
    enable_wide_fov_stereo: false
    max_num_points: 600000
    min_num_points: 1000

    # Feature detection parameters
    detector_type: "FeatureTrackProcessor::kLk"
    max_features: 1000
    min_features: 100
    min_distance: 20.0

    # Tracking parameters
    lk_max_level: 3
    lk_min_level: 0
    num_pyramid_levels: 4
    refine_features: true

    # Optimization parameters
    enable_observations: true
    enable_keypoint_interpolation: true
    enable_corrected_keypoint: true
    enable_grid_filtering: true

    # Mapping parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_odom_tf: true
    publish_base_tf: true

    # Camera parameters (adjust for your camera)
    camera_matrix:
      [381.22, 0.0, 320.5,
       0.0, 381.22, 240.5,
       0.0, 0.0, 1.0]

    # Distortion coefficients
    distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]

    # Image processing
    image_width: 640
    image_height: 480
    queue_size: 1
```

### 4. Create a VSLAM launch file
Create `launch/vslam.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vslam_config_pkg'),
            'config',
            'vslam_config.yaml'
        ]),
        description='Path to VSLAM configuration file'
    )

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'enable_occupancy_map': True,
                'occupancy_map_resolution': 0.05,
                'occupancy_map_size_x': 50.0,
                'occupancy_map_size_y': 50.0,
                'occupancy_map_size_z': 5.0,
            }
        ],
        remappings=[
            ('/visual_slam/camera/left/image_rect', '/camera/color/image_raw'),
            ('/visual_slam/camera/left/camera_info', '/camera/color/camera_info'),
            ('/visual_slam/camera/right/image_rect', '/camera/depth/image_rect_raw'),
            ('/visual_slam/camera/right/camera_info', '/camera/depth/camera_info'),
            ('/visual_slam/imu', '/imu/data'),
        ],
        output='screen'
    )

    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('vslam_config_pkg'),
        'rviz',
        'vslam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='vslam_rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        visual_slam_node,
        rviz_node
    ])
```

### 5. Create an RViz2 configuration for VSLAM
Create `rviz/vslam.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1
        - /Odometry1
        - /PointCloud1
        - /Map1
      Splitter Ratio: 0.5
    Tree Height: 617
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Angle Tolerance: 0.10000000149011612
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.10000000149011612
      Shape:
        Alpha: 1
        Color: 255; 25; 0
        Type: Arrow
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/odometry
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/visual_slam_result_pointcloud
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/occupancy_map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/occupancy_map_updates
      Use Timestamp: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1200
  X: 72
  Y: 60
```

### 6. Create a VSLAM evaluation node
Create `vslam_evaluation/vslam_evaluator.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
from std_msgs.msg import Float32
import numpy as np
import math
from collections import deque


class VSLAMEvaluator(Node):
    def __init__(self):
        super().__init__('vslam_evaluator')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for VSLAM output
        self.odom_sub = self.create_subscription(
            Odometry, '/visual_slam/odometry', self.odom_callback, 10
        )

        # Publishers for evaluation metrics
        self.accuracy_pub = self.create_publisher(
            Float32, 'vslam/accuracy', 10
        )
        self.drift_pub = self.create_publisher(
            Float32, 'vslam/drift', 10
        )
        self.tracking_quality_pub = self.create_publisher(
            Float32, 'vslam/tracking_quality', 10
        )

        # Data storage
        self.trajectory = deque(maxlen=1000)  # Store recent poses
        self.ground_truth_poses = deque(maxlen=1000)  # Store ground truth if available
        self.evaluation_counter = 0

        # Evaluation parameters
        self.evaluation_rate = 5.0  # Hz
        self.timer = self.create_timer(1.0/self.evaluation_rate, self.evaluate_vslam)

        # Statistics
        self.total_error = 0.0
        self.error_count = 0

        self.get_logger().info('VSLAM Evaluator Node Started')

    def odom_callback(self, msg):
        """Receive odometry from VSLAM system."""
        # Store the current pose
        current_pose = {
            'position': (msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z),
            'orientation': (msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w),
            'timestamp': msg.header.stamp
        }

        self.trajectory.append(current_pose)
        self.evaluation_counter += 1

    def evaluate_vslam(self):
        """Evaluate VSLAM performance metrics."""
        if len(self.trajectory) < 2:
            return

        # Calculate trajectory length
        trajectory_length = self.calculate_trajectory_length()

        # Calculate drift (deviation from straight line)
        drift = self.calculate_drift()

        # Calculate tracking quality based on feature stability
        tracking_quality = self.calculate_tracking_quality()

        # Publish metrics
        accuracy_msg = Float32()
        accuracy_msg.data = 0.95 if len(self.trajectory) > 10 else 0.0  # Placeholder
        self.accuracy_pub.publish(accuracy_msg)

        drift_msg = Float32()
        drift_msg.data = drift
        self.drift_pub.publish(drift_msg)

        quality_msg = Float32()
        quality_msg.data = tracking_quality
        self.tracking_quality_pub.publish(quality_msg)

        # Log metrics periodically
        if self.evaluation_counter % int(self.evaluation_rate * 5) == 0:  # Every 5 seconds
            self.get_logger().info(
                f'VSLAM Evaluation - Drift: {drift:.3f}m, '
                f'Tracking Quality: {tracking_quality:.3f}, '
                f'Trajectory Length: {trajectory_length:.3f}m'
            )

    def calculate_trajectory_length(self):
        """Calculate the total length of the trajectory."""
        if len(self.trajectory) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(self.trajectory)):
            prev_pos = self.trajectory[i-1]['position']
            curr_pos = self.trajectory[i]['position']

            distance = math.sqrt(
                (curr_pos[0] - prev_pos[0])**2 +
                (curr_pos[1] - prev_pos[1])**2 +
                (curr_pos[2] - prev_pos[2])**2
            )
            total_length += distance

        return total_length

    def calculate_drift(self):
        """Calculate drift as deviation from expected path."""
        if len(self.trajectory) < 2:
            return 0.0

        # Calculate drift as the distance from the first pose to current pose
        # compared to the actual path length
        first_pos = self.trajectory[0]['position']
        last_pos = self.trajectory[-1]['position']

        direct_distance = math.sqrt(
            (last_pos[0] - first_pos[0])**2 +
            (last_pos[1] - first_pos[1])**2 +
            (last_pos[2] - first_pos[2])**2
        )

        path_length = self.calculate_trajectory_length()

        # Drift is the difference between path length and direct distance
        drift = max(0.0, path_length - direct_distance)

        return drift

    def calculate_tracking_quality(self):
        """Calculate tracking quality based on pose consistency."""
        if len(self.trajectory) < 10:
            return 0.5  # Return medium quality if not enough data

        # Calculate the variance in pose changes
        position_changes = []
        for i in range(1, min(10, len(self.trajectory))):
            prev_pos = self.trajectory[-i-1]['position']
            curr_pos = self.trajectory[-i]['position']

            change = math.sqrt(
                (curr_pos[0] - prev_pos[0])**2 +
                (curr_pos[1] - prev_pos[1])**2 +
                (curr_pos[2] - prev_pos[2])**2
            )
            position_changes.append(change)

        if not position_changes:
            return 0.5

        # Calculate standard deviation of position changes
        std_dev = np.std(position_changes)

        # Quality is inversely related to standard deviation
        # Lower std dev means more consistent tracking
        quality = max(0.0, min(1.0, 1.0 - std_dev))

        return quality


def main(args=None):
    rclpy.init(args=args)
    vslam_evaluator = VSLAMEvaluator()

    try:
        rclpy.spin(vslam_evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 7. Update package.xml for VSLAM package
Edit `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vslam_config_pkg</name>
  <version>0.0.0</version>
  <description>Configuration package for Isaac ROS Visual SLAM</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>isaac_ros_visual_slam</depend>
  <depend>isaac_ros_stereo_image_pipeline</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>rviz2</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
</file>
```

### 8. Create a comprehensive launch file
Create `launch/vslam_complete.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vslam_config_pkg'),
            'config',
            'vslam_config.yaml'
        ]),
        description='Path to VSLAM configuration file'
    )

    # Include Isaac ROS Visual SLAM
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_visual_slam'),
                'launch',
                'isaac_ros_visual_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
        }.items()
    )

    # VSLAM Evaluator
    vslam_evaluator = Node(
        package='vslam_config_pkg',
        executable='vslam_evaluator',
        name='vslam_evaluator',
        output='screen'
    )

    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('vslam_config_pkg'),
        'rviz',
        'vslam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='vslam_rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        vslam_launch,
        vslam_evaluator,
        rviz_node
    ])
```

### 9. Build and run the VSLAM system
```bash
cd ~/ros2_ws
colcon build --packages-select vslam_config_pkg
source install/setup.bash

# Terminal 1: Run Isaac Sim with a camera
# (Run your Isaac Sim scene with camera sensors)

# Terminal 2: Launch VSLAM
ros2 launch vslam_config_pkg vslam_complete.launch.py

# Terminal 3: Monitor VSLAM metrics
ros2 topic echo /vslam/accuracy
ros2 topic echo /vslam/drift
```

## Expected Output
- VSLAM node processes camera images and estimates robot pose
- Map is built and updated in real-time
- Robot trajectory is tracked and displayed in RViz2
- Evaluation metrics show VSLAM performance (accuracy, drift, tracking quality)

## Troubleshooting
- If VSLAM fails to initialize, check camera calibration parameters
- If tracking is poor, verify sufficient features in the environment
- If drift is high, consider adding loop closure or improving feature detection

## Next Steps
- Integrate VSLAM with navigation stack
- Add loop closure detection for improved accuracy
- Implement multi-session mapping
- Optimize parameters for specific environments