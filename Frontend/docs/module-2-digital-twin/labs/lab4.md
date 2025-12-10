---
sidebar_position: 4
---

# Lab 4: Exporting Gazebo Environment to Unity

## Overview

In this lab, you'll learn how to export your Gazebo simulation environment to Unity for high-fidelity visualization. This creates a digital twin that combines Gazebo's accurate physics simulation with Unity's advanced rendering capabilities, providing both realistic physics and photorealistic visualization.

## Prerequisites

- Lab 1-3 completed (Gazebo, humanoid model, and sensors configured)
- Unity Hub and Unity 2022.3 LTS or newer installed
- Unity Robotics Simulation package
- ROS 2 Humble Hawksbill installed
- Basic familiarity with Unity interface

## Learning Objectives

By the end of this lab, you will be able to:
- Understand the Gazebo to Unity export workflow
- Install and configure Unity Robotics Simulation package
- Export a Gazebo environment to Unity-compatible format
- Import and set up the exported environment in Unity
- Configure basic Unity settings for robotics simulation

## Background

The Gazebo ↔ Unity pipeline enables a powerful combination for robotics development:
- **Gazebo**: Provides accurate physics simulation and sensor modeling
- **Unity**: Offers high-fidelity graphics rendering and immersive visualization
- **ROS 2**: Acts as the communication layer between both environments

This approach allows for "photorealistic simulation" where you can train perception algorithms on realistic visual data before deploying to real robots.

## Installing Unity and Required Packages

### Step 1: Install Unity Hub and Unity Editor

1. Download Unity Hub from https://unity.com/download
2. Install Unity Hub and create an account if needed
3. Through Unity Hub, install Unity 2022.3 LTS or newer
4. Make sure to include the "Universal Render Pipeline" and "XR/VR" packages during installation

### Step 2: Install Unity Robotics Simulation Package

1. Open Unity Hub and create a new 3D project
2. In the Unity Editor, open the Package Manager (Window → Package Manager)
3. In the Package Manager, click the "+" button and select "Add package from git URL..."
4. Add the Unity Robotics Simulation package:
   - For the core package: `com.unity.robotics.simulation`
   - You may also want to add: `com.unity.robotics.urdf-importer` for URDF support

### Step 3: Install ROS# Package (Optional but Recommended)

ROS# enables direct communication between Unity and ROS 2:

1. Download the ROS# Unity package from the Unity Asset Store or GitHub
2. Import it into your Unity project via Assets → Import Package → Custom Package

## Exporting Gazebo Environment to Unity-Compatible Format

### Step 4: Prepare Your Gazebo Environment for Export

The process of exporting from Gazebo to Unity is not direct. Instead, we'll follow a workflow that involves:

1. Converting SDF models to URDF (if not already done)
2. Using the URDF Importer in Unity to bring in robot models
3. Recreating the environment in Unity with similar physics properties

Let's start by preparing your environment:

1. Ensure your world file is well-structured and contains all necessary models
2. Make sure all custom models have proper SDF files
3. Note down the positions and orientations of all objects in your Gazebo world

### Step 5: Export Robot Model as URDF

Your humanoid robot model should already be in URDF format from Lab 2. If not, convert your SDF model to URDF:

```bash
# If you have an SDF model, you can convert it using various tools
# For this lab, we'll use the URDF from previous labs
```

### Step 6: Export Environment Objects

For static objects in your environment (like the ground plane, tables, etc.), you'll need to recreate them in Unity. Here's how to document your environment:

1. List all objects in your Gazebo world file
2. Note their positions, rotations, and scales
3. Save their mesh files if they're custom models

## Setting Up Unity for Robotics Simulation

### Step 7: Create a New Unity Project

1. Open Unity Hub
2. Create a new 3D project (name it "HumanoidRobotSimulation" or similar)
3. Wait for the project to initialize

### Step 8: Import the URDF Importer Package

1. In your Unity project, go to Assets → Import Package → Custom Package
2. Import the URDF Importer package if you downloaded it separately
3. The URDF Importer allows you to directly import your robot model into Unity

### Step 9: Import Your Robot Model

1. Create a folder in your Unity project: `Assets/Robots/Humanoid`
2. Copy your URDF file and all associated mesh files to this folder
3. In Unity, select your URDF file
4. Unity should automatically import the robot using the URDF Importer
5. The robot will appear in your scene with all its joints and visual elements

### Step 10: Recreate the Environment

Now recreate your Gazebo environment in Unity:

1. Create a ground plane (GameObject → 3D Object → Plane)
2. Adjust its scale to match your Gazebo world (typically scale it to 20x20 units)
3. Add a material with a grid texture for visual reference
4. Recreate other objects (tables, obstacles) using Unity's primitive objects or import custom models

Here's a basic setup for the environment:

```csharp
// Create a script to set up the environment
// Save as EnvironmentSetup.cs in Assets/Scripts/

using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    [Header("Environment Settings")]
    public float groundSize = 20f;
    public Color groundColor = new Color(0.7f, 0.7f, 0.7f, 1f);

    [Header("Objects")]
    public GameObject[] staticObstacles;

    void Start()
    {
        SetupGround();
        SetupObstacles();
    }

    void SetupGround()
    {
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.name = "Ground";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(groundSize / 10f, 1, groundSize / 10f);

        // Add a grid material
        Renderer groundRenderer = ground.GetComponent<Renderer>();
        groundRenderer.material = new Material(Shader.Find("Standard"));
        groundRenderer.material.color = groundColor;
    }

    void SetupObstacles()
    {
        // Add any static obstacles here based on your Gazebo world
        foreach (GameObject obstacle in staticObstacles)
        {
            // Instantiate obstacles at appropriate positions
            if (obstacle != null)
            {
                Instantiate(obstacle, obstacle.transform.position, obstacle.transform.rotation);
            }
        }
    }
}
```

### Step 11: Configure Physics Settings

1. Go to Edit → Project Settings → Physics
2. Adjust the physics settings to match Gazebo's physics behavior:
   - Set Gravity to (0, -9.81, 0) to match Earth's gravity
   - Adjust Default Material properties (bounciness, friction) to match your Gazebo models
   - Set Fixed Timestep to match Gazebo's update rate (typically 0.001 for 1000 Hz)

### Step 12: Set Up Lighting

For a realistic simulation environment:

1. Remove the default Directional Light
2. Add a new Directional Light (GameObject → Light → Directional Light)
3. Set the rotation to (50, -30, 0) to match typical Gazebo lighting
4. Adjust the intensity to around 1.0
5. Consider adding additional lights for better visualization

## Connecting Unity to ROS 2

### Step 13: Set Up ROS 2 Communication

To enable communication between Unity and ROS 2:

1. Install the ROS TCP Connector package in Unity
2. Create a ROS Connection Manager in your scene:
   - Create an empty GameObject (GameObject → Create Empty)
   - Name it "ROSConnection"
   - Add the RosConnection script component

3. Configure the ROS Connection:
   - Set the ROS IP to "127.0.0.1" (localhost) or your ROS 2 master IP
   - Set the port to 10000 (or your configured port)

### Step 14: Create ROS Message Publishers/Subscribers

For your humanoid robot with sensors, create appropriate publishers and subscribers:

```csharp
// Example script for publishing joint states
// Save as JointStatePublisher.cs in Assets/Scripts/

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class JointStatePublisher : MonoBehaviour
{
    [SerializeField] private List<Transform> jointTransforms = new List<Transform>();
    [SerializeField] private List<string> jointNames = new List<string>();

    private ROSConnection ros;
    private JointStateMsg jointStateMsg;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("joint_states");

        jointStateMsg = new JointStateMsg();
        jointStateMsg.name = jointNames.ToArray();
    }

    void FixedUpdate()
    {
        // Update joint positions
        jointStateMsg.position = new double[jointTransforms.Count];
        for (int i = 0; i < jointTransforms.Count; i++)
        {
            // Convert Unity rotation to joint angle (simplified)
            jointStateMsg.position[i] = jointTransforms[i].localEulerAngles.y;
        }

        jointStateMsg.header = new HeaderMsg();
        jointStateMsg.header.stamp = new TimeMsg();
        jointStateMsg.header.stamp.sec = (int)Time.time;
        jointStateMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);

        ros.Publish("joint_states", jointStateMsg);
    }
}
```

## Verification and Testing

### Step 15: Test the Unity Environment

1. In Unity, press Play to start the simulation
2. Verify that:
   - The ground plane appears correctly
   - Your humanoid robot model is imported and visible
   - All joints move properly
   - Physics behave as expected

### Step 16: Test ROS 2 Connection

1. Start your ROS 2 environment in another terminal
2. Launch a simple ROS 2 node to verify communication
3. Check that Unity can send and receive messages from ROS 2

```bash
# In a terminal, verify ROS 2 topics
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Advanced Unity Configuration

### Using Unity's Universal Render Pipeline (URP)

For better visual quality:

1. Go to Assets → Create → Rendering → Universal Render Pipeline → Pipeline Asset
2. In Project Settings → Graphics, assign the new URP asset
3. Create a URP_Renderer asset and assign it to the pipeline asset

### Adding Post-Processing Effects

For more realistic rendering:

1. Add the Post-Processing package through Package Manager
2. Create a Post-Process Volume in your scene
3. Add effects like Bloom, Color Grading, and Ambient Occlusion

### Setting Up Cameras

For various viewpoints:

1. Create multiple cameras (Main Camera, Front Camera, Top Camera)
2. Set different layers and culling masks
3. Configure camera properties to match real robot sensors if needed

## Troubleshooting Common Issues

### Issue 1: URDF Import Problems
**Solution**: Check that all mesh files referenced in the URDF are in the correct location and in a supported format (FBX, OBJ, etc.).

### Issue 2: Physics Behavior Mismatch
**Solution**: Adjust Unity's physics settings to better match Gazebo's physics parameters, including gravity, friction, and bounciness.

### Issue 3: ROS 2 Connection Failures
**Solution**:
- Ensure ROS 2 network is properly configured
- Check firewall settings
- Verify ROS TCP Connector is properly configured

### Issue 4: Performance Issues
**Solution**:
- Reduce polygon count of meshes
- Use Level of Detail (LOD) groups
- Optimize lighting and post-processing effects

## Exporting and Sharing

### Step 17: Build Your Unity Application

1. Go to File → Build Settings
2. Select your target platform (Windows, Linux, etc.)
3. Add your scenes to the build
4. Configure build settings for your deployment needs
5. Build and run to create a standalone application

## Summary

In this lab, you've learned how to export your Gazebo environment to Unity, creating a digital twin that combines accurate physics simulation with high-fidelity visualization. You've set up the Unity environment, imported your robot model, configured physics settings, and established ROS 2 communication. This creates a powerful platform for robotics development and testing.

## Next Steps

In Lab 5, you'll create a complete digital twin scene with two-way ROS communication, integrating all the components you've learned about in this module to create a fully functional simulation system.