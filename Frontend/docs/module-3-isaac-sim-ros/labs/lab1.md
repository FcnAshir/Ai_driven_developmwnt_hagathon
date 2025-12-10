---
sidebar_position: 1
---

# Lab 1: Install Isaac Sim on RTX Workstation

## Overview

This lab will guide you through installing NVIDIA Isaac Sim on an RTX-enabled workstation. Isaac Sim is a robotics simulator built on NVIDIA Omniverse that provides photorealistic simulation environments for developing and testing AI-based robotics applications.

## Prerequisites

- NVIDIA RTX GPU (RTX 3080 or higher recommended)
- Ubuntu 22.04 LTS
- NVIDIA GPU drivers (535 or higher)
- CUDA 12.x installed
- Docker and Docker Compose
- At least 32GB RAM (64GB recommended)
- 100GB+ free disk space
- ROS 2 Humble Hawksbill installed

## Learning Objectives

By the end of this lab, you will be able to:
- Verify system requirements for Isaac Sim
- Install NVIDIA Omniverse and Isaac Sim
- Configure Isaac Sim for robotics simulation
- Launch Isaac Sim and verify basic functionality
- Understand the Isaac Sim interface and navigation

## System Requirements Verification

### Step 1: Check GPU and Driver Compatibility

First, verify that your system has the necessary hardware and drivers:

```bash
# Check GPU information
nvidia-smi

# Verify CUDA installation
nvcc --version

# Check for RTX compatibility
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
```

### Step 2: Install Required Dependencies

Install the necessary system dependencies:

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y build-essential cmake git python3-dev python3-pip \
    python3-venv python3-setuptools python3-wheel python3-numpy \
    libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1 \
    libgtk-3-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libavdevice-dev

# Install Docker if not already installed
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
```

### Step 3: Verify ROS 2 Installation

Ensure ROS 2 Humble is properly installed:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

## Installing Isaac Sim

### Step 4: Download Isaac Sim

Visit the NVIDIA Isaac Sim download page and download the appropriate version for your system. You'll need to create an NVIDIA Developer account if you don't have one.

For this lab, we'll use the Omniverse launcher approach:

```bash
# Download Omniverse launcher
wget https://developer.download.nvidia.com/devzone/redist/omniverse_launcher/linux_x64/omniverse_launcher.AppImage

# Make it executable
chmod +x omniverse_launcher.AppImage

# Run the launcher
./omniverse_launcher.AppImage
```

### Step 5: Install Isaac Sim through Omniverse

1. Launch the Omniverse App Launcher
2. Sign in with your NVIDIA Developer account
3. Search for "Isaac Sim" in the app store
4. Click "Install" to download and install Isaac Sim
5. Wait for the installation to complete (this may take 20-30 minutes)

### Step 6: Configure Isaac Sim

After installation, you'll need to configure Isaac Sim for robotics development:

1. Launch Isaac Sim from the Omniverse launcher
2. Accept the license agreement
3. Configure the cache directory (recommended: at least 50GB free space)
4. Set up the extensions directory

## Initial Isaac Sim Setup

### Step 7: Verify Installation

Once Isaac Sim is launched:

1. You should see the Isaac Sim welcome screen
2. Check that the interface loads properly
3. Verify that you can navigate the interface using the controls:
   - Orbit: Right mouse button + drag
   - Pan: Middle mouse button + drag
   - Zoom: Mouse wheel or Shift + right mouse button + drag

### Step 8: Basic Scene Test

Test basic functionality:

1. Create a new scene (File → New)
2. Add a simple primitive (Create → Mesh → Cube)
3. Verify you can select and manipulate the object
4. Try basic rendering by pressing the Play button

## Isaac Sim for Robotics Configuration

### Step 9: Install Robotics Extensions

Isaac Sim comes with several robotics-specific extensions. Enable them:

1. Go to Window → Extensions
2. In the "Isaac" section, enable:
   - Isaac Sim Robotics Apps
   - Isaac Sim Sensors
   - Isaac Sim Navigation
   - Isaac Sim Perception
3. Restart Isaac Sim after enabling extensions

### Step 10: Configure Physics Settings

For robotics simulation, adjust the physics settings:

1. Go to Window → Physics
2. Set the gravity to (0, 0, -9.81) for Earth-like gravity
3. Adjust solver settings for robotics applications:
   - Solver Type: TGS (Truncated Generalized Solver)
   - Substeps: 8-16 for stable simulation
   - Time Step: 1/60 or 1/100 for real-time simulation

## Docker-Based Isaac Sim (Alternative Installation)

If you prefer a containerized installation, you can use Isaac Sim Docker containers:

```bash
# Pull the Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Run Isaac Sim in Docker (requires NVIDIA Container Toolkit)
docker run --gpus all -it --rm \
  --network=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER/.Xauthority:/root/.Xauthority:rw" \
  --volume="/home/$USER/isaac_sim_data:/isaac_sim_data" \
  --privileged \
  --shm-size="1gb" \
  nvcr.io/nvidia/isaac-sim:4.0.0
```

## Troubleshooting Common Issues

### Issue 1: GPU Not Detected
**Symptoms**: Isaac Sim fails to launch or runs with poor performance
**Solution**:
- Verify NVIDIA drivers are properly installed: `nvidia-smi`
- Ensure the correct CUDA version is installed
- Check that the GPU meets minimum requirements

### Issue 2: Memory Issues
**Symptoms**: Crashes or poor performance during simulation
**Solution**:
- Close unnecessary applications
- Increase virtual memory if needed
- Consider using simpler scenes for testing

### Issue 3: Rendering Problems
**Symptoms**: Artifacts, poor rendering quality, or crashes
**Solution**:
- Update graphics drivers
- Check X11 forwarding settings if using remote access
- Verify OpenGL support

### Issue 4: Extension Loading Issues
**Symptoms**: Robotics extensions fail to load
**Solution**:
- Verify Isaac Sim is properly licensed
- Check internet connection for extension downloads
- Restart Isaac Sim after enabling extensions

## Verification

To verify that Isaac Sim is properly installed and configured:

1. Isaac Sim launches without errors
2. The interface loads properly with all robotics extensions enabled
3. You can create and manipulate basic objects
4. Physics simulation runs smoothly
5. Rendering quality is acceptable for robotics simulation

## Performance Optimization

For optimal performance with robotics simulation:

1. **Scene Complexity**: Start with simple scenes and gradually increase complexity
2. **Render Quality**: Adjust quality settings based on your hardware capabilities
3. **Physics Settings**: Balance accuracy with performance requirements
4. **Memory Management**: Monitor memory usage during simulation

## Summary

In this lab, you've successfully installed NVIDIA Isaac Sim on your RTX-enabled workstation and configured it for robotics simulation. You've learned about the system requirements, installation process, and basic configuration for robotics applications. This foundation will be essential for the more advanced robotics simulations and perception tasks in the following labs.

## Next Steps

Now that Isaac Sim is installed and configured, proceed to Lab 2 where you'll build your first Isaac ROS perception pipeline for robotics applications.