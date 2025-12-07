---
sidebar_position: 5
---

# Lab 5: Creating a Digital Twin Scene with Two-Way ROS Communication

## Overview

In this final lab of Module 2, you'll create a complete digital twin scene that enables two-way communication between Gazebo physics simulation and Unity visualization via ROS 2. This creates a comprehensive simulation environment where commands sent from ROS 2 affect both the physics simulation and the visual representation, while sensor data flows back to ROS 2 from both environments.

## Prerequisites

- Lab 1-4 completed (Gazebo, humanoid model, sensors, Unity export)
- ROS 2 Humble Hawksbill installed
- Unity with Robotics packages installed
- Basic understanding of ROS 2 communication patterns
- Completed all previous labs in Module 2

## Learning Objectives

By the end of this lab, you will be able to:
- Set up bidirectional communication between Gazebo and Unity
- Implement ROS 2 bridges for synchronized simulation
- Create a complete digital twin system with synchronized physics and visualization
- Test and validate the digital twin system functionality
- Understand the benefits and limitations of the Gazebo-Unity digital twin approach

## Background

A digital twin in robotics creates a virtual replica of a physical system that can mirror its behavior in real-time. In our case, we're creating a digital twin that combines:

1. **Gazebo**: Accurate physics simulation and sensor modeling
2. **Unity**: High-fidelity visualization and user interaction
3. **ROS 2**: Communication layer enabling synchronization

The two-way communication means:
- Commands from ROS 2 affect both Gazebo physics and Unity visualization
- Sensor data flows from both environments back to ROS 2
- State changes in either environment are reflected in the other

## System Architecture

```
[ROS 2 Nodes] ←→ [Gazebo Bridge] ←→ [Gazebo Physics]
     ↑                     ↓
[Unity Bridge] ←→ [Unity Visualization]
```

## Setting Up Two-Way Communication

### Step 1: Install ROS-Gazebo-Unity Bridge

First, install the necessary packages for bridging all three systems:

```bash
# Install ROS 2 Gazebo bridge
sudo apt install ros-humble-ros-gz-bridge

# Install additional packages for Unity integration
sudo apt install ros-humble-ros-ign-bridge
```

### Step 2: Create Bridge Configuration Files

Create a bridge configuration file to specify which topics to synchronize between Gazebo and Unity.

Create a file `~/ros2_ws/src/humanoid_robot_description/config/digital_twin_bridge.yaml`:

```yaml
# Bridge configuration for Gazebo-Unity digital twin
bridge_name: "digital_twin_bridge"

# Topics to bridge from Gazebo to ROS 2
gazebo_to_ros:
  - topic_name: "/scan"
    ros_type: "sensor_msgs/msg/LaserScan"
    gz_type: "gz.msgs.LaserScan"
    direction: "GZ_TO_ROS"

  - topic_name: "/imu/data"
    ros_type: "sensor_msgs/msg/Imu"
    gz_type: "gz.msgs.IMU"
    direction: "GZ_TO_ROS"

  - topic_name: "/camera/rgb/image_raw"
    ros_type: "sensor_msgs/msg/Image"
    gz_type: "gz.msgs.Image"
    direction: "GZ_TO_ROS"

  - topic_name: "/camera/depth/image_raw"
    ros_type: "sensor_msgs/msg/Image"
    gz_type: "gz.msgs.Image"
    direction: "GZ_TO_ROS"

  - topic_name: "/joint_states"
    ros_type: "sensor_msgs/msg/JointState"
    gz_type: "gz.msgs.Model"
    direction: "GZ_TO_ROS"

# Topics to bridge from ROS 2 to Gazebo
ros_to_gazebo:
  - topic_name: "/cmd_vel"
    ros_type: "geometry_msgs/msg/Twist"
    gz_type: "gz.msgs.Twist"
    direction: "ROS_TO_GZ"

  - topic_name: "/joint_commands"
    ros_type: "std_msgs/msg/Float64MultiArray"
    gz_type: "gz.msgs.DoubleArray"
    direction: "ROS_TO_GZ"

# Topics to bridge between Unity and ROS 2
unity_to_ros:
  - topic_name: "/unity_robot_state"
    ros_type: "sensor_msgs/msg/JointState"
    unity_type: "JointState"
    direction: "UNITY_TO_ROS"

  - topic_name: "/unity_camera_image"
    ros_type: "sensor_msgs/msg/Image"
    unity_type: "Image"
    direction: "UNITY_TO_ROS"

ros_to_unity:
  - topic_name: "/unity_robot_commands"
    ros_type: "std_msgs/msg/Float64MultiArray"
    unity_type: "JointCommand"
    direction: "ROS_TO_UNITY"
```

### Step 3: Create a Comprehensive Launch File

Create a launch file that starts all components simultaneously. Create `~/ros2_ws/src/humanoid_robot_description/launch/digital_twin.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('humanoid_robot_description'),
        'worlds',
        'humanoid_world.sdf'
    ))

    # Start Gazebo with the world file
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Robot state publisher
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_robot_description'),
        'urdf',
        'simple_humanoid_sensors.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_humanoid_sensors'],
        output='screen'
    )

    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/joint_commands@std_msgs/msg/Float64MultiArray@gz.msgs.DoubleArray'
        ],
        remappings=[
            ('/scan', '/lidar_scan'),
            ('/imu/data', '/imu_data'),
        ],
        output='screen'
    )

    # Unity bridge node (placeholder - actual implementation depends on your Unity setup)
    unity_bridge = Node(
        package='humanoid_robot_description',
        executable='unity_bridge_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # TF2 static transform publisher for Unity coordinate system
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'unity_world'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        unity_bridge,  # This would be your custom Unity bridge node
        tf_publisher,
    ])
```

### Step 4: Create Unity ROS Bridge Script

In your Unity project, create a script to handle ROS communication. Create `Assets/Scripts/DigitalTwinBridge.cs`:

```csharp
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.BuiltinInterfaces;

public class DigitalTwinBridge : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIpAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Configuration")]
    public Transform robotRoot;
    public List<Transform> jointTransforms = new List<Transform>();
    public List<string> jointNames = new List<string>();

    [Header("Sensors")]
    public Camera rgbCamera;
    public Camera depthCamera;
    public Transform imuTransform;

    private ROSConnection ros;
    private float publishRate = 0.1f; // 10 Hz
    private float lastPublishTime = 0;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIpAddress, rosPort);

        // Subscribe to robot commands
        ros.Subscribe<Float64MultiArrayMsg>("/unity_robot_commands", ReceiveRobotCommands);

        // Set up publishers
        InvokeRepeating(nameof(PublishJointStates), 0, publishRate);
        InvokeRepeating(nameof(PublishImuData), 0, publishRate);
        InvokeRepeating(nameof(PublishCameraData), 0, publishRate);
    }

    void ReceiveRobotCommands(Float64MultiArrayMsg commands)
    {
        // Process commands to move robot joints
        if (commands.data.Length == jointTransforms.Count)
        {
            for (int i = 0; i < jointTransforms.Count; i++)
            {
                // Apply command to joint (simplified - actual implementation depends on joint type)
                float targetAngle = (float)commands.data[i];

                // For revolute joints, you might use:
                // jointTransforms[i].localRotation = Quaternion.Euler(0, targetAngle, 0);
            }
        }
    }

    void PublishJointStates()
    {
        JointStateMsg jointState = new JointStateMsg();
        jointState.name = jointNames.ToArray();
        jointState.position = new double[jointTransforms.Count];
        jointState.velocity = new double[jointTransforms.Count];
        jointState.effort = new double[jointTransforms.Count];

        for (int i = 0; i < jointTransforms.Count; i++)
        {
            // Convert Unity rotation to joint angles
            jointState.position[i] = jointTransforms[i].localEulerAngles.y;
            jointState.velocity[i] = 0; // Simplified
            jointState.effort[i] = 0;   // Simplified
        }

        jointState.header = new HeaderMsg();
        jointState.header.stamp = new TimeMsg();
        jointState.header.stamp.sec = (int)Time.time;
        jointState.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);

        ros.Publish("/unity_joint_states", jointState);
    }

    void PublishImuData()
    {
        ImuMsg imuMsg = new ImuMsg();

        // Convert Unity accelerometer data to ROS IMU message
        imuMsg.linear_acceleration.x = imuTransform.InverseTransformDirection(Physics.gravity).x;
        imuMsg.linear_acceleration.y = imuTransform.InverseTransformDirection(Physics.gravity).y;
        imuMsg.linear_acceleration.z = imuTransform.InverseTransformDirection(Physics.gravity).z;

        // Angular velocity (simplified)
        imuMsg.angular_velocity.x = 0;
        imuMsg.angular_velocity.y = 0;
        imuMsg.angular_velocity.z = 0;

        // Orientation (simplified - would need proper quaternion conversion)
        imuMsg.orientation.x = imuTransform.rotation.x;
        imuMsg.orientation.y = imuTransform.rotation.y;
        imuMsg.orientation.z = imuTransform.rotation.z;
        imuMsg.orientation.w = imuTransform.rotation.w;

        imuMsg.header = new HeaderMsg();
        imuMsg.header.stamp = new TimeMsg();
        imuMsg.header.stamp.sec = (int)Time.time;
        imuMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);

        ros.Publish("/unity_imu_data", imuMsg);
    }

    void PublishCameraData()
    {
        // Capture and publish camera data
        if (rgbCamera != null)
        {
            // This is a simplified example - actual image capture and publishing
            // would require more complex implementation
            ros.Publish("/unity_rgb_image", new ImageMsg());
        }

        if (depthCamera != null)
        {
            ros.Publish("/unity_depth_image", new ImageMsg());
        }
    }

    void Update()
    {
        // Synchronize with ROS time if needed
        if (Time.time - lastPublishTime > publishRate)
        {
            lastPublishTime = Time.time;
        }
    }
}
```

### Step 5: Create Unity Scene Setup

Create a scene setup script that initializes the digital twin environment. Create `Assets/Scripts/DigitalTwinSetup.cs`:

```csharp
using UnityEngine;

public class DigitalTwinSetup : MonoBehaviour
{
    [Header("Environment Configuration")]
    public float simulationScale = 1.0f; // Scale factor for Unity to real-world conversion
    public bool synchronizePhysics = true;
    public float physicsUpdateRate = 100f; // Hz

    [Header("Synchronization Settings")]
    public float maxSyncDelay = 0.1f; // Maximum allowed delay in seconds
    public bool autoSync = true;

    [Header("Debug Settings")]
    public bool showDebugInfo = true;

    private DigitalTwinBridge bridge;
    private float lastSyncTime;
    private float syncInterval = 0.01f; // Sync every 10ms

    void Start()
    {
        // Find the bridge component
        bridge = FindObjectOfType<DigitalTwinBridge>();
        if (bridge == null)
        {
            Debug.LogError("DigitalTwinBridge component not found in the scene!");
            return;
        }

        // Configure Unity physics to match Gazebo
        if (synchronizePhysics)
        {
            ConfigurePhysics();
        }

        lastSyncTime = Time.time;
    }

    void ConfigurePhysics()
    {
        // Set Unity physics to match Gazebo settings
        Physics.gravity = new Vector3(0, -9.81f, 0) * simulationScale;

        // Set fixed timestep to match Gazebo's update rate
        Time.fixedDeltaTime = 1.0f / physicsUpdateRate;
    }

    void Update()
    {
        if (autoSync && Time.time - lastSyncTime > syncInterval)
        {
            SynchronizeEnvironment();
            lastSyncTime = Time.time;
        }

        if (showDebugInfo)
        {
            DisplayDebugInfo();
        }
    }

    void SynchronizeEnvironment()
    {
        // This method would handle synchronization between Unity and Gazebo
        // In practice, this would involve receiving state updates from ROS
        // and applying them to Unity objects, or sending Unity state to ROS
    }

    void DisplayDebugInfo()
    {
        // Display synchronization status and other debug information
        if (showDebugInfo)
        {
            Debug.Log($"Digital Twin Status: Active | Physics Rate: {1.0f / Time.fixedDeltaTime} Hz");
        }
    }

    // Method to handle state synchronization from ROS
    public void ApplyStateFromROS(Vector3 position, Quaternion rotation)
    {
        transform.position = position * simulationScale;
        transform.rotation = rotation;
    }

    // Method to send state to ROS
    public void SendStateToROS()
    {
        // Send current Unity state to ROS via the bridge
        // This would include position, rotation, joint angles, etc.
    }
}
```

## Testing the Digital Twin System

### Step 6: Launch the Complete System

1. Build your Unity application:
   - File → Build Settings
   - Add your scene
   - Build for your target platform

2. Launch the ROS 2 and Gazebo components:
```bash
# Terminal 1: Start ROS 2 and Gazebo
cd ~/ros2_ws
source install/setup.bash
ros2 launch humanoid_robot_description digital_twin.launch.py
```

3. Run your Unity application separately or use the Unity editor in Play mode

4. Verify that:
   - Commands sent to ROS 2 affect both Gazebo and Unity
   - Sensor data from both environments is available in ROS 2
   - The simulation states remain synchronized

### Step 7: Test Communication

Create a simple test script to verify two-way communication:

```bash
# Terminal 2: Send a command to move the robot
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Terminal 3: Listen to sensor data
ros2 topic echo /lidar_scan
ros2 topic echo /unity_joint_states
```

## Advanced Digital Twin Features

### Adding Time Synchronization

For precise synchronization, implement time synchronization between all components:

```csharp
// Add to DigitalTwinBridge.cs
private double rosTimeOffset = 0;

void SyncWithROSTime(TimeMsg rosTime)
{
    double currentTime = Time.time;
    double rosTimeSec = rosTime.sec + rosTime.nanosec / 1e9;
    rosTimeOffset = currentTime - rosTimeSec;
}

double GetSynchronizedTime()
{
    return Time.time - rosTimeOffset;
}
```

### Implementing State Synchronization

Create a more sophisticated synchronization mechanism:

```csharp
[System.Serializable]
public class RobotState
{
    public string robotName;
    public Vector3 position;
    public Quaternion rotation;
    public List<double> jointPositions;
    public List<double> jointVelocities;
    public double timestamp;
}

public void SynchronizeRobotState(RobotState state)
{
    // Apply the state to Unity objects
    robotRoot.position = state.position;
    robotRoot.rotation = state.rotation;

    for (int i = 0; i < jointTransforms.Count && i < state.jointPositions.Count; i++)
    {
        // Apply joint positions
        float angle = (float)state.jointPositions[i];
        jointTransforms[i].localRotation = Quaternion.Euler(0, angle, 0);
    }
}
```

## Verification and Validation

### Step 8: Validate the Digital Twin System

To verify that your digital twin system is working correctly:

1. **State Consistency**: Check that robot states are consistent between Gazebo and Unity
2. **Sensor Data**: Verify that sensor data from both environments is available in ROS 2
3. **Command Response**: Ensure that commands sent to ROS 2 affect both environments
4. **Timing**: Verify that both environments run at consistent rates
5. **Synchronization**: Check that the simulation states remain synchronized over time

### Step 9: Performance Testing

Test the performance of your digital twin system:

```bash
# Monitor ROS 2 topic rates
ros2 run topic_tools relay /lidar_scan /monitored_scan

# Check system resource usage
htop
nvidia-smi  # if using GPU
```

## Troubleshooting Common Issues

### Issue 1: Synchronization Problems
**Solution**: Implement proper time synchronization and state reconciliation between environments.

### Issue 2: Performance Degradation
**Solution**:
- Optimize Unity rendering settings
- Reduce sensor update rates where possible
- Use efficient data structures for state synchronization

### Issue 3: Communication Failures
**Solution**:
- Check network connectivity between components
- Verify ROS 2 network configuration
- Ensure all bridge nodes are running properly

### Issue 4: Physics Mismatch
**Solution**: Fine-tune Unity physics parameters to match Gazebo settings more closely.

## Benefits and Limitations

### Benefits of the Digital Twin Approach
- **Safety**: Test algorithms without risking physical hardware
- **Cost-Effectiveness**: Reduce need for expensive physical prototypes
- **Rapid Iteration**: Quickly test multiple scenarios
- **Training Data**: Generate large datasets for AI training
- **Validation**: Validate systems before real-world deployment

### Limitations
- **Simulation Fidelity**: Physics and sensor models may not perfectly match reality
- **Computational Overhead**: Running multiple simulation environments requires significant resources
- **Synchronization Complexity**: Maintaining consistency between environments can be challenging
- **Latency**: Communication between components may introduce delays

## Summary

In this lab, you've created a comprehensive digital twin system that combines Gazebo's accurate physics simulation with Unity's high-fidelity visualization, all connected through ROS 2 for two-way communication. This system enables advanced robotics development, testing, and validation in a safe, controlled environment that closely mirrors real-world conditions.

The digital twin approach provides a powerful platform for developing and testing humanoid robotics applications before deploying to physical hardware, significantly reducing development time and risk.

## Next Steps

With Module 2 complete, you now have:
- A solid understanding of digital twin concepts
- Experience with Gazebo physics simulation
- Knowledge of Unity visualization techniques
- Skills in multi-environment synchronization
- A working digital twin system

In Module 3, you'll explore NVIDIA Isaac Sim and Isaac ROS, which build on these concepts with more advanced simulation capabilities specifically designed for AI-powered robotics.