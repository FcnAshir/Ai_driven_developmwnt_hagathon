---
sidebar_position: 3
---

# Lab 3: Adding LiDAR, IMU, and Depth Camera to Humanoid Robot

## Overview

In this lab, you'll learn how to add various sensors to your humanoid robot model in Gazebo, including LiDAR, IMU, and depth camera. These sensors are essential for perception capabilities in robotics, allowing the robot to understand its environment and navigate safely.

## Prerequisites

- Lab 1 completed (Gazebo installed and configured)
- Lab 2 completed (Humanoid URDF model loaded in Gazebo)
- ROS 2 Humble Hawksbill installed
- Basic understanding of ROS 2 topics and messages

## Learning Objectives

By the end of this lab, you will be able to:
- Add LiDAR sensor to a humanoid robot model
- Add IMU sensor to a humanoid robot model
- Add depth camera to a humanoid robot model
- Verify that sensors publish data correctly in ROS 2
- Understand the parameters and configuration of different sensor types

## Background

Sensors are critical components of any robotic system. In simulation, we model these sensors to generate realistic data that can be used for algorithm development before deployment on real hardware. The three sensor types we'll focus on are:

1. **LiDAR**: Provides 2D or 3D distance measurements for mapping and navigation
2. **IMU**: Measures acceleration and angular velocity for orientation and motion detection
3. **Depth Camera**: Provides 3D point cloud data and depth information for object recognition

## Adding Sensors to Your URDF Model

### Step 1: Update Your URDF Model

First, let's enhance the humanoid URDF model from Lab 2 to include sensors. Update your URDF file to include sensor definitions:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5"/>
  </joint>

  <!-- Head with sensors -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <!-- LiDAR Sensor on head -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_to_lidar" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0.0 0.0 0.08"/>
  </joint>

  <!-- IMU Sensor in torso -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_imu" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.0"/>
  </joint>

  <!-- Depth Camera in head -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.04 0.06 0.02"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.06 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_to_camera" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0.0 0.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_to_left_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_upper_arm_to_lower_arm" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_to_right_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_upper_arm_to_lower_arm" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="torso_to_left_upper_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 -0.3"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_upper_leg_to_lower_leg" type="fixed">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="torso_to_right_upper_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 -0.3"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_upper_leg_to_lower_leg" type="fixed">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="gz-sim-lidar-system">
        <topic>/scan</topic>
        <frame_id>lidar_link</frame_id>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>/imu</topic>
      <plugin name="imu_controller" filename="gz-sim-imu-system">
        <topic>/imu/data</topic>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth_camera">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="gz-sim-rgbd-camera-system">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/rgb</topic_name>
        <depth_topic_name>camera/depth</depth_topic_name>
        <point_cloud_topic_name>camera/points</point_cloud_topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find ros2_control_demo_example_1)/config/rrbot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="lidar_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_upper_leg">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="left_lower_leg">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_lower_leg">
    <material>Gazebo/Black</material>
  </gazebo>

</robot>
```

### Step 2: Update the Launch File

Update your launch file to use the new URDF with sensors. Create a new launch file named `spawn_humanoid_sensors.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_robot_description'),
        'urdf',
        'simple_humanoid_sensors.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher node
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_humanoid_sensors'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
    ])
```

## Testing Sensor Data

### Step 3: Launch the Simulation with Sensors

1. First, copy the updated URDF to your robot description package:

```bash
cp ~/ros2_ws/src/humanoid_robot_description/urdf/simple_humanoid_sensors.urdf ~/ros2_ws/src/humanoid_robot_description/urdf/simple_humanoid.urdf
```

2. Launch Gazebo with your world file:

```bash
# Terminal 1: Launch Gazebo
gz sim -r ~/ros2_ws/src/humanoid_robot_description/worlds/humanoid_world.sdf
```

3. In another terminal, spawn the robot with sensors:

```bash
# Terminal 2: Spawn the robot with sensors
cd ~/ros2_ws
source install/setup.bash
ros2 launch humanoid_robot_description spawn_humanoid_sensors.launch.py
```

### Step 4: Verify Sensor Data

Once the simulation is running, verify that sensors are publishing data:

1. Check available ROS 2 topics:

```bash
ros2 topic list
```

You should see topics like:
- `/scan` (LiDAR data)
- `/imu/data` (IMU data)
- `/camera/rgb/image_raw` (RGB camera data)
- `/camera/depth/image_raw` (Depth camera data)
- `/camera/points` (Point cloud data)

2. View LiDAR data:

```bash
# Terminal 3: View LiDAR scan data
ros2 topic echo /scan sensor_msgs/msg/LaserScan
```

3. View IMU data:

```bash
# Terminal 4: View IMU data
ros2 topic echo /imu/data sensor_msgs/msg/Imu
```

4. View camera data:

```bash
# Terminal 5: View RGB camera data
ros2 topic echo /camera/rgb/image_raw sensor_msgs/msg/Image
```

## Sensor Configuration Parameters

### LiDAR Configuration
- `update_rate`: How frequently the sensor updates (Hz)
- `samples`: Number of rays in the horizontal scan
- `min_angle` and `max_angle`: Angular range of the scan
- `range`: Minimum and maximum detection distance

### IMU Configuration
- `update_rate`: How frequently the sensor updates (Hz)
- Provides linear acceleration, angular velocity, and orientation

### Depth Camera Configuration
- `update_rate`: How frequently the sensor updates (Hz)
- `horizontal_fov`: Horizontal field of view
- `image`: Resolution and format of the captured images
- Provides RGB images, depth images, and point clouds

## Troubleshooting Common Issues

### Issue 1: Sensor topics not appearing
**Solution**: Check that the Gazebo plugins are properly configured in your URDF and that the Gazebo ROS packages are installed:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Issue 2: Sensor data has high noise or unrealistic values
**Solution**: Add noise models to your sensor configuration:

```xml
<gaussian_noise>0.005</gaussian_noise>
```

### Issue 3: Simulation runs slowly with sensors
**Solution**: Reduce the update rates of sensors or decrease the resolution of cameras.

## Advanced Sensor Configurations

### Adding Multiple LiDAR Units

For a more comprehensive perception system, you might want multiple LiDAR sensors:

```xml
<!-- Front LiDAR -->
<joint name="head_to_front_lidar" type="fixed">
  <parent link="head"/>
  <child link="front_lidar_link"/>
  <origin xyz="0.05 0.0 0.0"/>
</joint>

<!-- Side LiDAR -->
<joint name="head_to_side_lidar" type="fixed">
  <parent link="head"/>
  <child link="side_lidar_link"/>
  <origin xyz="0.0 0.05 0.0" rpy="0 0 1.57"/>
</joint>
```

### Adding Other Sensor Types

You can also add other sensor types like GPS, force/torque sensors, or contact sensors depending on your application requirements.

## Verification

To verify that your sensors are working correctly:

1. All sensor topics should be available in ROS 2
2. Sensor data should update at the expected rate
3. Data should be within expected ranges (e.g., IMU values around 9.81 m/s² for gravity)
4. Visualization tools like RViz2 should be able to display sensor data

## Summary

In this lab, you've successfully added LiDAR, IMU, and depth camera sensors to your humanoid robot model in Gazebo. You've learned how to configure these sensors, verify their data publication, and understand the key parameters for each sensor type. These perception capabilities are essential for autonomous navigation and environment interaction in robotics applications.

## Next Steps

In Lab 4, you'll learn how to export your Gazebo environment to Unity for high-fidelity visualization and create a digital twin of your humanoid robot system.