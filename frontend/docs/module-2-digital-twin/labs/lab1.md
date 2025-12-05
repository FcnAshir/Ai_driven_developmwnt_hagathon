---
sidebar_position: 1
---

# Lab 1: Setting up Gazebo on Ubuntu 22.04

## Overview

This lab will guide you through installing and configuring Gazebo simulation environment on Ubuntu 22.04. Gazebo is a powerful physics-based simulation engine that provides realistic simulation of robots and their environments.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed (completed Module 1)
- Administrative access to install packages
- Minimum 8GB RAM and 40GB free disk space

## Learning Objectives

By the end of this lab, you will be able to:
- Install Gazebo Garden on Ubuntu 22.04
- Verify the installation by launching the Gazebo GUI
- Understand the basic Gazebo interface and controls
- Configure basic simulation parameters

## Installation Steps

### Step 1: Update System Packages

First, update your system's package list:

```bash
sudo apt update
sudo apt upgrade
```

### Step 2: Install Gazebo Garden

For Ubuntu 22.04, we'll install Gazebo Garden (the latest stable version as of 2024):

```bash
sudo apt install gazebo
```

Alternatively, if the above doesn't work, you can install it from the osrf repository:

```bash
curl -sSL http://get.gazebosim.org | sh
sudo apt install gz-garden
```

### Step 3: Install ROS 2 Gazebo Integration

Install the ROS 2 packages that enable integration between ROS 2 and Gazebo:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
sudo apt install ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Step 4: Install Additional Dependencies

Install additional packages that are commonly needed for robotics simulation:

```bash
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
```

### Step 5: Verify Installation

Test that Gazebo is properly installed by launching the GUI:

```bash
gz sim
```

If this is your first time running Gazebo, you might see a welcome screen. You can close it and explore the interface.

## Basic Gazebo Interface

When you launch Gazebo, you'll see several main components:

1. **Menu Bar**: File, Edit, View, Tools, Window, Help
2. **Toolbar**: Common actions like play/pause simulation, reset, etc.
3. **Scene View**: 3D visualization of the simulation environment
4. **Scene Tree**: List of objects in the current scene
5. **Inspector**: Properties of selected objects
6. **Layers**: Different visualization layers

## Basic Simulation Controls

- **Play/Pause**: Start or pause the physics simulation
- **Step**: Advance the simulation by one time step
- **Reset**: Reset the simulation to initial conditions
- **Follow**: Follow a specific object in the view
- **Select**: Select objects in the scene
- **Move/Rotate**: Move or rotate selected objects

## Creating Your First Simulation

Let's create a simple simulation environment:

1. Close the Gazebo GUI if it's running
2. Create a simple world file:

```bash
mkdir -p ~/gazebo_worlds
touch ~/gazebo_worlds/my_first_world.sdf
```

3. Edit the world file with a basic empty world:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
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
  </world>
</sdf>
```

4. Launch Gazebo with your custom world:

```bash
gz sim ~/gazebo_worlds/my_first_world.sdf
```

## Troubleshooting Common Issues

### Issue 1: Gazebo fails to launch with graphics errors
**Solution**: This is often related to graphics drivers. Try running with software rendering:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
gz sim
```

### Issue 2: Missing ROS 2 packages
**Solution**: Make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

### Issue 3: Performance issues or slow simulation
**Solution**: Adjust the real-time update rate in your world file or reduce visual complexity.

## Verification

To verify that your installation is working properly:

1. Gazebo GUI launches without errors
2. You can see the default empty world
3. The simulation runs at approximately real-time speed
4. You can interact with the interface (zoom, pan, rotate view)

## Summary

In this lab, you've successfully installed Gazebo Garden on Ubuntu 22.04 and verified that it works correctly. You've learned about the basic Gazebo interface and created your first simple simulation world. This foundation will be essential for the more complex simulations we'll create in subsequent labs.

## Next Steps

Now that you have Gazebo installed and running, proceed to Lab 2 where you'll learn to load humanoid URDF models into Gazebo for simulation.