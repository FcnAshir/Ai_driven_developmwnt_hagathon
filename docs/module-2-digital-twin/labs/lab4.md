# Lab 4: Export Environment to Unity

## Objective
Learn to transfer simulated environments from Gazebo to Unity for high-fidelity visualization.

## Prerequisites
- Unity Hub and Unity 2022.3 LTS installed
- Completed Lab 3 (Sensors in Gazebo)
- Basic understanding of 3D modeling and Unity interface
- Gazebo and ROS 2 properly configured

## Steps

### 1. Export Gazebo world as SDF and meshes
First, let's prepare our Gazebo world for export by creating a more complex environment:

Create `~/gazebo_worlds/unity_export_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="unity_export_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple room structure -->
    <model name="room_walls">
      <pose>0 0 2 0 0 0</pose>
      <link name="room_link">
        <visual name="room_visual">
          <geometry>
            <box>
              <size>8 8 4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="room_collision">
          <geometry>
            <box>
              <size>8 8 4</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1000.0</mass>
          <inertia>
            <ixx>1000.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1000.0</iyy>
            <iyz>0</iyz>
            <izz>1000.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a hole in the center to see inside -->
    <model name="room_with_hole">
      <pose>0 0 2 0 0 0</pose>
      <link name="outer_wall">
        <visual name="outer_visual">
          <geometry>
            <box>
              <size>8 8 4</size>
            </box>
          </geometry>
        </visual>
        <collision name="outer_collision">
          <geometry>
            <box>
              <size>8 8 4</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name="inner_cutout">
        <visual name="inner_visual">
          <geometry>
            <box>
              <size>6 6 4.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0 0</ambient>
            <diffuse>0 0 0 0</diffuse>
          </material>
        </visual>
        <collision name="inner_collision">
          <geometry>
            <box>
              <size>6 6 4.1</size>
            </box>
          </geometry>
        </collision>
      </link>
      <!-- Create a joint to position the cutout -->
      <joint name="cutout_joint" type="fixed">
        <parent>outer_wall</parent>
        <child>inner_cutout</child>
      </joint>
    </model>

    <!-- Add furniture -->
    <model name="table">
      <pose>-2 -1 0.4 0 0 0</pose>
      <link name="table_top">
        <visual name="table_visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="table_collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.2</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg1">
        <visual name="leg1_visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1</diffuse>
          </material>
        </visual>
        <collision name="leg1_collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.7</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="leg1_joint" type="fixed">
        <parent>table_top</parent>
        <child>leg1</child>
        <origin xyz="0.7 0.35 -0.375" rpy="0 0 0"/>
      </joint>
    </model>

    <!-- Add objects on the table -->
    <model name="cup">
      <pose>-2.2 -1.1 0.45 0 0 0</pose>
      <link name="cup_link">
        <visual name="cup_visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.9 1</ambient>
            <diffuse>0.8 0.8 0.9</diffuse>
          </material>
        </visual>
        <collision name="cup_collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0001</iyy>
            <iyz>0</iyz>
            <izz>0.000025</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a humanoid robot -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### 2. Export Gazebo models as meshes
Create a script to export the Gazebo models as OBJ files that can be imported into Unity:

Create `scripts/export_meshes.py`:

```python
#!/usr/bin/env python3
"""
This script demonstrates how to extract mesh data from Gazebo models.
In practice, you would use this to export meshes for Unity import.
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path


def extract_meshes_from_sdf(sdf_file_path, output_dir):
    """
    Extract mesh information from an SDF file and prepare for export.
    """
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    print(f"Extracting meshes from {sdf_file_path}")

    # Find all models with visual elements
    for model in root.findall('.//model'):
        model_name = model.get('name')
        print(f"Processing model: {model_name}")

        # Find all links in the model
        for link in model.findall('link'):
            link_name = link.get('name')

            # Find all visuals in the link
            for visual in link.findall('visual'):
                visual_name = visual.get('name')

                # Find geometry information
                geometry = visual.find('geometry')
                if geometry is not None:
                    # Check for different geometry types
                    box = geometry.find('box')
                    cylinder = geometry.find('cylinder')
                    sphere = geometry.find('sphere')

                    if box is not None:
                        size = box.find('size').text
                        print(f"  Box geometry: {size}")
                    elif cylinder is not None:
                        radius = cylinder.find('radius').text
                        length = cylinder.find('length').text
                        print(f"  Cylinder geometry: radius={radius}, length={length}")
                    elif sphere is not None:
                        radius = sphere.find('radius').text
                        print(f"  Sphere geometry: radius={radius}")

                    # Look for mesh geometry
                    mesh = geometry.find('mesh')
                    if mesh is not None:
                        uri = mesh.find('uri').text
                        print(f"  Mesh URI: {uri}")
                        # In practice, you would extract the actual mesh file here


def create_unity_import_guide():
    """
    Create a guide for importing the exported meshes into Unity.
    """
    guide_content = """
# Unity Import Guide for Gazebo Models

## Prerequisites
- Unity 2022.3 LTS or later
- Unity Robotics Hub package
- Unity Simulation package (optional)

## Steps

### 1. Install Unity Robotics Hub
1. Open Unity Hub and create a new project
2. In Unity, go to Window > Package Manager
3. Click the + button and select "Add package from git URL..."
4. Add the following packages:
   - com.unity.robotics.ros-tcp-connector
   - com.unity.robotics.urdf-importer

### 2. Import Gazebo Models
1. Create a new folder in Assets called "GazeboModels"
2. Copy your exported mesh files (.obj, .fbx, .dae) to this folder
3. Unity will automatically import and process the meshes

### 3. Set up the Environment
1. Create an empty GameObject as the root for your environment
2. Add individual mesh GameObjects as children
3. Set appropriate materials and textures

### 4. Configure Physics
1. Add Colliders to your mesh GameObjects for physics interaction
2. Set appropriate Rigidbody components if objects should be dynamic
3. Configure physics materials for realistic interaction

### 5. Set up Lighting
1. Add Directional Light to simulate Gazebo's sun
2. Configure lighting to match Gazebo's lighting conditions
3. Add additional lights as needed for high-fidelity rendering

### 6. Camera Setup
1. Create a camera to match Gazebo's default view
2. Configure camera properties (FOV, clipping planes) to match Gazebo sensors
3. Add post-processing effects for high-fidelity rendering

## ROS Integration
To maintain communication between Gazebo and Unity:

1. Use ROS TCP Connector for communication
2. Synchronize transforms between Gazebo and Unity
3. Forward sensor data from Gazebo to Unity representations
4. Send control commands from Unity back to Gazebo

## Best Practices
- Use appropriate scale (Gazebo typically uses meters)
- Ensure consistent coordinate systems (Z-up in Gazebo vs Y-up in Unity)
- Optimize meshes for real-time rendering
- Use LOD (Level of Detail) for complex models
    """

    with open("unity_import_guide.md", "w") as f:
        f.write(guide_content)

    print("Created unity_import_guide.md")


if __name__ == "__main__":
    # Example usage
    sdf_path = "~/gazebo_worlds/unity_export_world.sdf"  # Replace with actual path
    output_dir = "./exported_meshes"

    # Note: This is a conceptual script - actual mesh extraction would require
    # more complex processing of Gazebo's model database
    print("This script demonstrates the concept of mesh extraction.")
    print("Actual implementation would interface with Gazebo's model database.")
    create_unity_import_guide()
```

### 3. Create Unity project setup guide
Create `docs/unity_setup_guide.md`:

```markdown
# Unity Setup Guide for Gazebo Integration

## Overview
This guide explains how to set up a Unity project that can work in conjunction with Gazebo for high-fidelity visualization of robotic simulations.

## Prerequisites
- Unity Hub
- Unity 2022.3 LTS
- Unity Robotics Hub package
- Basic knowledge of Unity interface

## Step 1: Create Unity Project
1. Open Unity Hub
2. Click "New Project"
3. Select "3D (Built-in Render Pipeline)" template
4. Name your project (e.g., "GazeboUnityBridge")
5. Choose a location and click "Create"

## Step 2: Install Robotics Packages
1. In Unity, go to Window > Package Manager
2. Click the + button in the top-left corner
3. Select "Add package from git URL..."
4. Add the following packages:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`

## Step 3: Set Up Scene Structure
1. Create an empty GameObject called "Environment"
2. Create another empty GameObject called "Robot"
3. Organize your scene hierarchy for easy management

## Step 4: Import Gazebo Models
1. Export your Gazebo models as OBJ or FBX files
2. Copy the files to your Unity project's Assets folder
3. Unity will automatically import and process the models
4. Adjust scale and orientation as needed (Gazebo uses Z-up, Unity uses Y-up)

## Step 5: Configure Coordinate System
Gazebo and Unity use different coordinate systems:
- Gazebo: X-forward, Y-left, Z-up
- Unity: X-right, Y-up, Z-forward

To convert:
- Rotate models 90 degrees around the X-axis
- Scale appropriately (Gazebo meters vs Unity units)

## Step 6: Set Up Lighting
1. Add a Directional Light to represent Gazebo's sun
2. Configure the light to match Gazebo's lighting conditions:
   - Rotation: (50, -30, 0)
   - Color: White or light yellow
   - Intensity: 1-2

## Step 7: Add Materials and Textures
1. Create materials that match Gazebo's visual appearance
2. Apply appropriate textures for realistic rendering
3. Configure shaders for optimal performance

## Step 8: ROS Integration
1. Add ROS TCP Connector to your scene
2. Configure connection settings to match your ROS environment
3. Implement data synchronization between Gazebo and Unity

## Step 9: Camera Setup
1. Position a camera to match Gazebo's default view
2. Configure camera properties to match sensor specifications
3. Add multiple cameras for different viewpoints

## Best Practices
- Use appropriate scaling (1 Unity unit = 1 meter)
- Optimize meshes for real-time rendering
- Use occlusion culling for performance
- Implement Level of Detail (LOD) for complex models
- Test performance regularly
```

### 4. Create a ROS bridge node for Unity communication
Create `unity_bridge/unity_bridge_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
import json


class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_bridge_node')

        # Create publishers for Unity visualization
        self.unity_robot_state_pub = self.create_publisher(
            String, 'unity/robot_state', 10
        )
        self.unity_environment_pub = self.create_publisher(
            String, 'unity/environment_state', 10
        )

        # Create subscribers for Gazebo sensor data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu', self.imu_callback, 10
        )

        # TF broadcaster for Unity
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state tracking
        self.robot_joints = {}
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]

        # Timer for publishing Unity updates
        self.timer = self.create_timer(0.1, self.publish_unity_data)  # 10 Hz

        self.get_logger().info('Unity Bridge Node Started')

    def joint_callback(self, msg):
        """Update robot joint positions from Gazebo."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.robot_joints[name] = msg.position[i]

    def lidar_callback(self, msg):
        """Process LiDAR data for Unity visualization."""
        # For Unity visualization, we might send processed data
        pass

    def imu_callback(self, msg):
        """Process IMU data for Unity visualization."""
        self.robot_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def publish_unity_data(self):
        """Publish robot state data formatted for Unity."""
        # Create robot state message for Unity
        robot_state = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'joints': self.robot_joints,
            'position': self.robot_position,
            'orientation': self.robot_orientation
        }

        # Publish robot state
        robot_msg = String()
        robot_msg.data = json.dumps(robot_state)
        self.unity_robot_state_pub.publish(robot_msg)

        # Create environment state message
        env_state = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'objects': self.get_environment_objects()
        }

        # Publish environment state
        env_msg = String()
        env_msg.data = json.dumps(env_state)
        self.unity_environment_pub.publish(env_msg)

    def get_environment_objects(self):
        """Get static environment objects."""
        # In a real implementation, this would get objects from Gazebo
        return {
            'table': {
                'position': [-2.0, -1.0, 0.4],
                'orientation': [0, 0, 0, 1],
                'scale': [1.5, 0.8, 0.8]  # Adjust for Unity coordinate system
            },
            'cup': {
                'position': [-2.2, -1.1, 0.45],
                'orientation': [0, 0, 0, 1],
                'scale': [1, 1, 1]
            }
        }


def main(args=None):
    rclpy.init(args=args)
    unity_bridge = UnityBridgeNode()

    try:
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5. Create Unity C# script for receiving data
Create `unity_scripts/RobotController.cs` (as a text file for reference):

```csharp
// This is a C# script that would be used in Unity
// We'll create it as a text file for reference

using UnityEngine;
using System.Collections.Generic;

public class RobotController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public Dictionary<string, Transform> jointMap = new Dictionary<string, Transform>();

    [Header("ROS Connection")]
    public string rosIpAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot State")]
    private Dictionary<string, float> jointPositions = new Dictionary<string, float>();

    void Start()
    {
        // Initialize joint map (this would be set up in the Unity editor)
        InitializeJointMap();
    }

    void Update()
    {
        // Update joint positions based on received data
        UpdateRobotJoints();
    }

    private void InitializeJointMap()
    {
        // This would typically be configured in the Unity editor
        // For example:
        // jointMap["left_shoulder_joint"] = leftShoulderTransform;
        // jointMap["right_elbow_joint"] = rightElbowTransform;
    }

    public void UpdateJointPositions(Dictionary<string, float> positions)
    {
        // Update the stored joint positions
        foreach (var kvp in positions)
        {
            jointPositions[kvp.Key] = kvp.Value;
        }
    }

    private void UpdateRobotJoints()
    {
        // Apply joint positions to transforms
        foreach (var jointName in jointPositions.Keys)
        {
            if (jointMap.ContainsKey(jointName))
            {
                Transform jointTransform = jointMap[jointName];
                float angle = jointPositions[jointName];

                // Apply rotation (adjust axis as needed)
                jointTransform.localRotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

### 6. Update package.xml for the Unity bridge package
Create or update `package.xml` in a new package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>unity_bridge_pkg</name>
  <version>0.0.0</version>
  <description>Bridge package for Unity integration</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 7. Create a launch file for Unity bridge
Create `launch/unity_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unity_bridge_pkg',
            executable='unity_bridge_node',
            name='unity_bridge_node',
            output='screen',
            parameters=[
                {'update_rate': 10.0}  # 10 Hz update rate
            ]
        )
    ])
```

## Expected Output
- Gazebo world with complex environment and robot
- Unity project structure ready for high-fidelity visualization
- ROS bridge node publishing robot state data in Unity-compatible format
- Documentation for Unity setup and integration

## Troubleshooting
- If coordinate systems don't match, verify axis conversions between Gazebo and Unity
- If data isn't transferring, check ROS network configuration
- If Unity performance is poor, optimize meshes and reduce polygon count

## Next Steps
- Implement real-time synchronization between Gazebo and Unity
- Add advanced rendering features in Unity (shadows, reflections, etc.)
- Create a complete digital twin pipeline with bidirectional communication