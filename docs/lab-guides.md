# Hands-On Tutorials & Lab Guides

This section integrates all practical lab exercises from Modules 1 through 4, providing a coherent, step-by-step guide for building and experimenting with Physical AI and Humanoid Robotics systems. These labs are designed to be hands-on, allowing you to apply theoretical concepts in practical, simulated environments.

## Module 1: The Robotic Nervous System (ROS 2) Labs

-   **Lab 1: Create a ROS 2 Package**
    -   *Objective*: Learn to create and structure a basic ROS 2 package.
    -   *Location*: `docs/module-1-ros2/labs/lab1.md`

-   **Lab 2: Publish and Subscribe to Topics**
    -   *Objective*: Understand ROS 2 communication by implementing publishers and subscribers.
    -   *Location*: `docs/module-1-ros2/labs/lab2.md`

-   **Lab 3: Build a Humanoid URDF**
    -   *Objective*: Design and implement a Universal Robot Description Format (URDF) model for a humanoid robot.
    -   *Location*: `docs/module-1-ros2/labs/lab3.md`

-   **Lab 4: Control a Joint using rclpy**
    -   *Objective*: Programmatically control robot joints using the ROS 2 Python client library (`rclpy`).
    -   *Location*: `docs/module-1-ros2/labs/lab4.md`

-   **Lab 5: Connect a Local LLM Agent to ROS Actions**
    -   *Objective*: Integrate a local LLM for higher-level command interpretation and execution through ROS 2 actions.
    -   *Location*: `docs/module-1-ros2/labs/lab5.md`

## Module 2: The Digital Twin — Gazebo & Unity Simulation Environments Labs

-   **Lab 1: Set up Gazebo on Ubuntu 22.04**
    -   *Objective*: Install and configure the Gazebo physics simulator.
    -   *Location*: `docs/module-2-digital-twin/labs/lab1.md`

-   **Lab 2: Load Humanoid URDF in Gazebo**
    -   *Objective*: Spawn and simulate a humanoid robot model within Gazebo.
    -   *Location*: `docs/module-2-digital-twin/labs/lab2.md`

-   **Lab 3: Add LiDAR + IMU + Depth Camera**
    -   *Objective*: Integrate and simulate various sensors in Gazebo for realistic perception.
    -   *Location*: `docs/module-2-digital-twin/labs/lab3.md`

-   **Lab 4: Export Environment to Unity**
    -   *Objective*: Learn to transfer simulated environments from Gazebo to Unity for high-fidelity visualization.
    -   *Location*: `docs/module-2-digital-twin/labs/lab4.md`

-   **Lab 5: Create a Digital Twin Scene with Two-Way ROS Communication**
    -   *Objective*: Establish a functional digital twin with real-time ROS 2 communication between simulation and control systems.
    -   *Location*: `docs/module-2-digital-twin/labs/lab5.md`

## Module 3: The AI-Robot Brain — NVIDIA Isaac Simulation and Isaac ROS Labs

-   **Lab 1: Install Isaac Sim on RTX Workstation**
    -   *Objective*: Set up the NVIDIA Isaac Sim platform for advanced robotics simulation.
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab1.md`

-   **Lab 2: Build a Perception Pipeline**
    -   *Objective*: Implement a basic perception pipeline using Isaac ROS components.
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab2.md`

-   **Lab 3: Run Isaac ROS Visual SLAM**
    -   *Objective*: Utilize Isaac ROS for Visual Simultaneous Localization and Mapping (VSLAM).
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab3.md`

-   **Lab 4: Create a Nav2 Navigation Demo**
    -   *Objective*: Develop and test a navigation system using ROS 2 Nav2 stack.
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab4.md`

-   **Lab 5: Generate Synthetic Images for Training**
    -   *Objective*: Create synthetic datasets for training AI models using Isaac Sim.
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab5.md`

-   **Lab 6: Deploy ROS Nodes to Jetson Orin**
    -   *Objective*: Deploy and run ROS 2 applications on NVIDIA Jetson Orin embedded platforms.
    -   *Location*: `docs/module-3-isaac-sim-ros/labs/lab6.md`

## Module 4: Vision-Language-Action (VLA) for Humanoid Robotics Labs

-   **Lab 1: Whisper Installation + Voice Command**
    -   *Objective*: Install Whisper and transcribe voice commands to text.
    -   *Location*: `docs/module-4-vla/labs/lab1.md`

-   **Lab 2: Use LLM to Convert Text → ROS Action Plan**
    -   *Objective*: Generate robot action plans from natural language using an LLM.
    -   *Location*: `docs/module-4-vla/labs/lab2.md`

-   **Lab 3: Detect Objects using YOLO/Isaac ROS**
    -   *Objective*: Understand and simulate object detection for robotic perception.
    -   *Location*: `docs/module-4-vla/labs/lab3.md`

-   **Lab 4: Robot Picks an Object After Verbal Instruction**
    -   *Objective*: Simulate a robot picking an object based on integrated VLA commands.
    -   *Location*: `docs/module-4-vla/labs/lab4.md`

-   **Lab 5: Build Final VLA Project: “Clean the Room” Pipeline**
    -   *Objective*: Develop a comprehensive VLA project to simulate a complex task like "cleaning a room."
    -   *Location*: `docs/module-4-vla/labs/lab5.md`

## Conclusion

These lab guides provide the practical experience necessary to complement the theoretical knowledge gained in each module, leading towards the successful completion of the Capstone Project.
