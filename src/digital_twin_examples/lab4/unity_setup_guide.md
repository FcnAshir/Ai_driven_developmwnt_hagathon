# Unity ROS-TCP-Connector Setup Guide

## Prerequisites

- Unity 2022.3 LTS or later
- Unity Robotics Hub package
- Unity Simulation package (optional)

## Installation Steps

### 1. Install Unity Robotics Hub

1. Open Unity Hub and create a new project
2. In Unity, go to Window > Package Manager
3. Click the + button and select "Add package from git URL..."
4. Add the following packages:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`

### 2. Configure ROS-TCP-Connector

1. In Unity, go to GameObject > 3D Object > Cube to create a basic scene
2. Add the ROS Connection component:
   - Go to GameObject > Unity Robotics > ROS Connection
   - This creates a ROSConnection object in your scene
3. Configure the connection settings:
   - Host Name: `localhost`
   - TCP Port: `10000`

### 3. Create Unity Scene for Digital Twin

1. Create a new scene (File > New Scene)
2. Add the following components:
   - Main Camera (for visualization)
   - Directional Light (for lighting)
   - ROSConnection (for ROS communication)
   - Import your robot model

### 4. Set up Robot Visualization

1. Import your robot model (URDF or FBX format)
2. Create GameObjects for sensors (cameras, LiDAR, etc.)
3. Create scripts to handle ROS message subscriptions and publications

## Example Unity C# Scripts

### ROSConnection Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotTopic = "/unity_robot_command";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
    }

    // Update is called once per frame
    void Update()
    {
        // Send a message to a ROS topic
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ros.SendServiceMessage<RobotCommandMsg>(robotTopic,
                new RobotCommandMsg(),
                SendRobotCommandResponse);
        }
    }

    void SendRobotCommandResponse(RobotCommandMsg response)
    {
        Debug.Log("Robot command sent successfully");
    }
}
```

### Sensor Data Visualization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class SensorVisualizer : MonoBehaviour
{
    [SerializeField] private GameObject lidarPointPrefab;
    [SerializeField] private Transform lidarOrigin;

    void Start()
    {
        ROSConnection.instance.Subscribe<LaserScanMsg>("/humanoid_robot/scan",
            ReceiveLidarData);
    }

    void ReceiveLidarData(LaserScanMsg scan)
    {
        // Visualize LiDAR data in Unity
        for (int i = 0; i < scan.ranges.Length; i++)
        {
            float distance = scan.ranges[i];
            if (distance < scan.range_max && distance > scan.range_min)
            {
                Vector3 direction = Quaternion.Euler(0, i * scan.angle_increment * Mathf.Rad2Deg, 0) * transform.forward;
                Vector3 pointPosition = lidarOrigin.position + direction * distance;

                GameObject point = Instantiate(lidarPointPrefab, pointPosition, Quaternion.identity);
                Destroy(point, 0.1f);
            }
        }
    }
}
```

## Integration with ROS 2

The ROS-TCP-Connector enables bidirectional communication between Unity and ROS 2:

- Unity can publish ROS messages (e.g., robot commands)
- Unity can subscribe to ROS messages (e.g., sensor data)
- Unity can call ROS services
- Unity can use ROS actions

## Best Practices

1. **Coordinate Systems**: Ensure consistency between ROS (Z-up) and Unity (Y-up) coordinate systems
2. **Scale**: Use meters as the unit of measurement in both environments
3. **Performance**: Optimize mesh complexity for real-time rendering
4. **Synchronization**: Maintain timing consistency between simulation and Unity visualization
5. **Error Handling**: Implement robust error handling for network disconnections

## Troubleshooting

- If Unity cannot connect to ROS, ensure the TCP port is open and accessible
- Check that both Unity and ROS are running on the same network interface
- Verify that the ROS bridge node is properly configured and running
- Monitor Unity console for any error messages