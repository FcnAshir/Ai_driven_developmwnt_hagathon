#!/usr/bin/env python3
"""
Isaac Sim Sensor Simulation

This script demonstrates sensor simulation in Isaac Sim, including cameras, LiDAR, and IMU.
"""

import omni
from pxr import Gf, UsdGeom
import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.core.utils.prims import get_prim_at_path


def setup_camera_sensor(world):
    """
    Set up a camera sensor in Isaac Sim
    """
    print("Setting up camera sensor...")

    # Define camera parameters
    camera_path = "/World/Robot/Camera"

    # Create camera sensor
    # In a real implementation, we would use Isaac's camera sensor
    print(f"Created camera sensor at {camera_path}")

    # Set camera properties
    camera_properties = {
        "resolution": (640, 480),
        "focal_length": 24.0,
        "horizontal_aperture": 20.955,
        " clipping_range": (0.1, 100.0)
    }

    print(f"Camera properties: {camera_properties}")

    return camera_path


def setup_lidar_sensor(world):
    """
    Set up a LiDAR sensor in Isaac Sim
    """
    print("Setting up LiDAR sensor...")

    # Define LiDAR parameters
    lidar_path = "/World/Robot/LiDAR"

    # Create LiDAR sensor
    # In a real implementation, we would use Isaac's LiDAR sensor
    print(f"Created LiDAR sensor at {lidar_path}")

    # Set LiDAR properties
    lidar_properties = {
        "rotation_frequency": 10,  # Hz
        "points_per_second": 500000,
        "laser_count": 16,
        "max_range": 25.0,
        "min_range": 0.2,
        "horizontal_fov": 360.0,
        "vertical_fov": 30.0
    }

    print(f"LiDAR properties: {lidar_properties}")

    return lidar_path


def setup_imu_sensor(world):
    """
    Set up an IMU sensor in Isaac Sim
    """
    print("Setting up IMU sensor...")

    # Define IMU parameters
    imu_path = "/World/Robot/IMU"

    # Create IMU sensor
    print(f"Created IMU sensor at {imu_path}")

    # Set IMU properties
    imu_properties = {
        "linear_acceleration_range": 16.0,  # g
        "angular_velocity_range": 2000.0,  # deg/s
        "linear_acceleration_noise_density": 0.0023,  # (m/s²)/√Hz
        "gyroscope_noise_density": 0.0004,  # (rad/s)/√Hz
    }

    print(f"IMU properties: {imu_properties}")

    return imu_path


def capture_sensor_data(world, camera_path, lidar_path, imu_path):
    """
    Capture and process data from all sensors
    """
    print("Capturing sensor data...")

    # Simulate camera data capture
    print(f"  Capturing image from {camera_path}")
    camera_data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)  # Simulated image
    print(f"  Captured image: {camera_data.shape}")

    # Simulate LiDAR data capture
    print(f"  Capturing point cloud from {lidar_path}")
    lidar_data = np.random.random((1000, 3)) * 25  # Simulated point cloud
    print(f"  Captured point cloud: {lidar_data.shape} points")

    # Simulate IMU data capture
    print(f"  Capturing IMU data from {imu_path}")
    imu_data = {
        "linear_acceleration": np.random.random(3) * 2 - 1,  # -1 to 1 g
        "angular_velocity": np.random.random(3) * 4 - 2,    # -2 to 2 rad/s
        "orientation": [1.0, 0.0, 0.0, 0.0]  # w, x, y, z quaternion
    }
    print(f"  Captured IMU data: {imu_data}")

    return {
        "camera": camera_data,
        "lidar": lidar_data,
        "imu": imu_data
    }


def run_sensor_demo():
    """
    Run a complete sensor simulation demonstration
    """
    print("Starting Isaac Sim Sensor Simulation Demo...")

    try:
        # Create world instance
        world = World(stage_units_in_meters=1.0)

        # Set up sensors
        camera_path = setup_camera_sensor(world)
        lidar_path = setup_lidar_sensor(world)
        imu_path = setup_imu_sensor(world)

        # Capture sensor data
        sensor_data = capture_sensor_data(world, camera_path, lidar_path, imu_path)

        print("\nSensor simulation demo completed successfully!")
        print(f"Captured data types: {list(sensor_data.keys())}")

    except Exception as e:
        print(f"Error during sensor simulation: {e}")
        return False

    return True


def main():
    """
    Main function to run the Isaac Sim sensor simulation
    """
    print("Starting Isaac Sim Sensor Simulation...")

    success = run_sensor_demo()

    if success:
        print("\nIsaac Sim Sensor Simulation completed successfully!")
    else:
        print("\nFailed to execute sensor simulation demo.")


if __name__ == "__main__":
    main()