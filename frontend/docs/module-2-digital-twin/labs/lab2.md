---
sidebar_position: 2
---

# Lab 2: Loading Humanoid URDF in Gazebo

## Overview

In this lab, you'll learn how to load humanoid robot URDF models into Gazebo for simulation. This is a crucial step in creating realistic humanoid robot simulations, as the URDF (Unified Robot Description Format) defines the robot's physical structure, joints, and visual properties.

## Prerequisites

- Lab 1 completed (Gazebo installed and configured)
- ROS 2 Humble Hawksbill installed
- Basic understanding of URDF format (covered in Module 1)
- A humanoid URDF model file (either created or downloaded)

## Learning Objectives

By the end of this lab, you will be able to:
- Prepare a humanoid URDF model for Gazebo simulation
- Launch Gazebo with a humanoid robot model
- Verify that the robot model loads correctly with proper physics properties
- Control the robot's joints in simulation

## Background

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. When working with Gazebo, additional Gazebo-specific tags can be added to the URDF to define simulation properties like materials, physics, and plugins.

## Preparation Steps

### Step 1: Obtain or Create a Humanoid URDF Model

If you don't have a humanoid URDF model, you can use a simple example. For this lab, we'll create a basic humanoid model with:

- Torso
- Head
- Two arms (upper arm, lower arm, hand)
- Two legs (upper leg, lower leg, foot)

### Step 2: Create the URDF Model

Create a directory for your robot model:

```bash
mkdir -p ~/ros2_ws/src/humanoid_robot_description/urdf
cd ~/ros2_ws/src/humanoid_robot_description/urdf
```

Create a simple humanoid URDF file named `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

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

  <!-- Head -->
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

## Loading the Model into Gazebo

### Step 3: Create a Launch File

Create a launch file to spawn the robot in Gazebo. First, create the launch directory:

```bash
mkdir -p ~/ros2_ws/src/humanoid_robot_description/launch
```

Create a launch file named `spawn_humanoid.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_robot_description'),
        'urdf',
        'simple_humanoid.urdf'
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
        arguments=['-topic', 'robot_description', '-entity', 'simple_humanoid'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
    ])
```

### Step 4: Create a World File

Create a world file for your simulation in `~/ros2_ws/src/humanoid_robot_description/worlds/humanoid_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- A global light source -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 -0.4 -0.8</direction>
    </light>

    <!-- A ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Optional: Add some obstacles for the humanoid to interact with -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Step 5: Launch the Simulation

Now, let's launch Gazebo with your humanoid model:

1. First, make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_robot_description
source install/setup.bash
```

2. Launch Gazebo with the world file:

```bash
# Terminal 1: Launch Gazebo
gz sim -r ~/ros2_ws/src/humanoid_robot_description/worlds/humanoid_world.sdf
```

3. In another terminal, spawn the robot:

```bash
# Terminal 2: Spawn the robot
cd ~/ros2_ws
source install/setup.bash
ros2 launch humanoid_robot_description spawn_humanoid.launch.py
```

## Verification

To verify that your humanoid model loaded correctly:

1. The robot should appear in the Gazebo simulation
2. All links should be visible with appropriate colors
3. The robot should be stable (not falling through the ground)
4. Joint constraints should be respected (if using revolute joints)
5. You should be able to see the robot in the Scene Tree

## Troubleshooting Common Issues

### Issue 1: Robot falls through the ground
**Solution**: Check that the `<static>` tag is properly set for the ground plane, and that the robot's `<inertial>` tags have appropriate mass values.

### Issue 2: URDF parsing errors
**Solution**: Validate your URDF file using:
```bash
check_urdf ~/ros2_ws/src/humanoid_robot_description/urdf/simple_humanoid.urdf
```

### Issue 3: Robot appears but is invisible
**Solution**: Ensure that `<visual>` tags are properly defined for each link and that materials are specified.

### Issue 4: Joints don't move as expected
**Solution**: Check joint types, limits, and axes. For humanoid arms, revolute joints are typically used for shoulder and elbow joints.

## Advanced Considerations

### Adding Sensors to Your Humanoid Model

To make your humanoid model more useful for robotics applications, consider adding sensors:

```xml
<!-- Example: Adding an IMU to the torso -->
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Improving Physics Properties

For more realistic humanoid simulation, consider:
- More accurate inertial properties based on actual humanoid robot specifications
- Proper joint limits that reflect human-like ranges of motion
- Adding damping and friction to joints for more natural movement

## Summary

In this lab, you've successfully loaded a humanoid URDF model into Gazebo simulation. You've learned how to create a basic humanoid model with appropriate physical properties, set up the necessary launch files, and verify that the simulation runs correctly. This forms the foundation for more complex humanoid robotics simulations and control experiments.

## Next Steps

In Lab 3, you'll learn how to add various sensors to your humanoid robot model, including LiDAR, IMU, and depth cameras, which are essential for perception in robotics applications.