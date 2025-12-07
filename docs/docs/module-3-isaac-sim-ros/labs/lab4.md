---
sidebar_position: 4
---

# Lab 4: Create a Nav2 Navigation Demo

## Overview

In this lab, you'll learn how to create a complete navigation system using Navigation2 (Nav2) integrated with Isaac Sim and Isaac ROS. You'll set up autonomous navigation, configure path planning, and test navigation in simulated environments.

## Prerequisites

- Lab 1-3 completed (Isaac Sim, perception pipeline, Visual SLAM)
- Isaac ROS Visual SLAM running (from Lab 3)
- Navigation2 packages installed
- ROS 2 Humble Hawksbill installed
- Understanding of TF frames and coordinate systems

## Learning Objectives

By the end of this lab, you will be able to:
- Install and configure Navigation2 for Isaac Sim
- Set up autonomous navigation with SLAM maps
- Configure path planners and controllers
- Test navigation in simulated environments
- Evaluate navigation performance and reliability

## Background

Navigation2 (Nav2) is the state-of-the-art navigation stack for ROS 2 that provides:
- Global and local path planning
- Localization (AMCL)
- Trajectory control
- Recovery behaviors
- Behavior trees for complex navigation logic

When combined with Isaac Sim and Isaac ROS Visual SLAM, Nav2 enables:
- Simultaneous mapping and navigation
- Photorealistic sensor simulation
- Hardware-accelerated perception
- Realistic physics simulation

## Installing Navigation2

### Step 1: Install Navigation2 Packages

First, install the necessary Navigation2 packages:

```bash
# Update package list
sudo apt update

# Install Navigation2 packages
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-nav2-rviz-plugins
sudo apt install -y ros-humble-nav2-behavior-tree
sudo apt install -y ros-humble-nav2-controller-server
sudo apt install -y ros-humble-nav2-planner-server
sudo apt install -y ros-humble-nav2-recoveries
sudo apt install -y ros-humble-nav2-lifecycle-manager
sudo apt install -y ros-humble-nav2-amcl
sudo apt install -y ros-humble-nav2-map-server
sudo apt install -y ros-humble-nav2-simulator
```

### Step 2: Verify Installation

Check that Navigation2 packages are properly installed:

```bash
# Verify packages are installed
ros2 pkg list | grep nav2

# Check available launch files
find /opt/ros/humble/share/nav2_bringup/launch -name "*.py"
```

## Setting Up Navigation Configuration

### Step 3: Create Navigation Workspace

Create a workspace for navigation configuration:

```bash
# Create workspace for navigation
mkdir -p ~/isaac_nav2_ws/src
cd ~/isaac_nav2_ws

# Create a package for navigation configuration
cd src
ros2 pkg create --build-type ament_python nav2_isaac_examples
cd nav2_isaac_examples
```

### Step 4: Create Navigation Configuration Files

Create the main navigation configuration file. Create `~/isaac_nav2_ws/src/nav2_isaac_examples/config/nav2_isaac_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_remaining_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      target_yaw_tolerance: 0.785

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      drive_on_heading_dist: 0.5
      drive_on_heading_angle_tolerance: 0.785
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1s

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Step 5: Create Navigation Launch File

Create a launch file that integrates Isaac Sim, Visual SLAM, and Navigation2. Create `~/isaac_nav2_ws/src/nav2_isaac_examples/launch/nav2_isaac_slam.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(FindPackageShare('nav2_isaac_examples').find('nav2_isaac_examples'), 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(FindPackageShare('nav2_isaac_examples').find('nav2_isaac_examples'), 'config', 'nav2_isaac_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start the navigation stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # Lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server',
                                    'planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'waypoint_follower']}])

    # Launch the ROS 2 Navigation Stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level}.items())

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        lifecycle_manager,
        nav2_bringup_launch
    ])
```

### Step 6: Create Isaac Sim Integration Launch File

Create a launch file that integrates Isaac Sim with Navigation2. Create `~/isaac_nav2_ws/src/nav2_isaac_examples/launch/nav2_isaac_integration.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    run_slam = LaunchConfiguration('run_slam')

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start the navigation stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(FindPackageShare('nav2_isaac_examples').find('nav2_isaac_examples'), 'config', 'nav2_isaac_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    declare_run_slam_cmd = DeclareLaunchArgument(
        'run_slam',
        default_value='True',
        description='Whether to run SLAM in addition to navigation')

    # Include Isaac ROS Visual SLAM launch if requested
    visual_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('visual_slam_examples').find('visual_slam_examples'), 'launch', 'visual_slam.launch.py')),
        condition=IfCondition(run_slam),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items())

    # Navigation bringup
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('nav2_isaac_examples').find('nav2_isaac_examples'), 'launch', 'nav2_isaac_slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'False',  # We're using Visual SLAM instead of traditional SLAM
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level
        }.items())

    # TF broadcasters for coordinate system alignment
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Isaac Sim camera bridge (if needed)
    camera_bridge = Node(
        package='isaac_ros_image_proc',
        executable='image_format_converter_node',
        name='image_format_converter_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_encoding': 'rgb8',
            'output_encoding': 'bgr8'
        }],
        remappings=[
            ('image_raw', '/camera/rgb/image_raw'),
            ('image', '/camera/rgb/image_rect_color')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_run_slam_cmd,
        visual_slam_launch,
        navigation_launch,
        tf_broadcaster,
        camera_bridge
    ])
```

### Step 7: Create Package Configuration

Create the package configuration file `~/isaac_nav2_ws/src/nav2_isaac_examples/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'nav2_isaac_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_isaac_slam.launch.py', 'launch/nav2_isaac_integration.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_isaac_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Isaac Sim Navigation2 Integration Examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

Create the package manifest `~/isaac_nav2_ws/src/nav2_isaac_examples/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nav2_isaac_examples</name>
  <version>0.0.1</version>
  <description>Isaac Sim Navigation2 Integration Examples</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>

  <exec_depend>navigation2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>isaac_ros_visual_slam</exec_depend>
  <exec_depend>isaac_ros_common</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
</xml>
```

## Building and Testing Navigation

### Step 8: Build the Workspace

```bash
cd ~/isaac_nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_isaac_examples
source install/setup.bash
```

### Step 9: Set Up Isaac Sim Scene for Navigation

1. Launch Isaac Sim
2. Create a scene with a mobile robot (e.g., wheeled robot with cameras and LiDAR)
3. Configure the robot with appropriate sensors:
   - RGB camera for Visual SLAM
   - Stereo cameras if using stereo Visual SLAM
   - LiDAR for obstacle detection (if needed)
4. Set up proper coordinate frames and TF tree

### Step 10: Run the Navigation Demo

```bash
# Terminal 1: Launch Isaac Sim with your navigation scene
# (Run Isaac Sim separately)

# Terminal 2: Launch the integrated navigation system
source ~/isaac_nav2_ws/install/setup.bash
source ~/isaac_vslam_ws/install/setup.bash  # From Lab 3
ros2 launch nav2_isaac_examples nav2_isaac_integration.launch.py

# Terminal 3: Send navigation goals (in RViz2 or via command line)
source /opt/ros/humble/setup.bash
# Example: Send a goal to (1, 1) with orientation (0, 0, 0, 1)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Advanced Navigation Configuration

### Step 11: Custom Behavior Trees

Create custom behavior trees for specific navigation scenarios:

```xml
<!-- Example: navigate_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="6" name="NavigateRecovery">
            <PipelineSequence name="NavigateWithReplanning">
                <RateController hz="1.0">
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                </RateController>
                <FollowPath path="{path}" controller_id="FollowPath"/>
            </PipelineSequence>
            <ReactiveFallback name="NavigateRecoveryFallback">
                <GoalUpdated/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
                <RecoveryNode number_of_retries="1" name="Spin">
                    <Spin spin_dist="1.57"/>
                </RecoveryNode>
            </ReactiveFallback>
        </RecoveryNode>
    </BehaviorTree>
</root>
```

### Step 12: Path Planner Configuration

Fine-tune path planners for your specific environment:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true  # Use A* instead of Dijkstra
      allow_unknown: true
      visualize_potential: false
```

## Testing Navigation Scenarios

### Step 13: Basic Navigation Test

1. Set up a simple environment in Isaac Sim
2. Start the navigation system
3. Send a navigation goal to a known location
4. Observe path planning and execution
5. Verify the robot reaches the goal successfully

### Step 14: Obstacle Avoidance Test

1. Place obstacles in the robot's path in Isaac Sim
2. Send a navigation goal that requires obstacle avoidance
3. Observe the robot's ability to replan and navigate around obstacles
4. Verify successful navigation to the goal

### Step 15: Recovery Behavior Test

1. Create a scenario where the robot gets stuck
2. Verify that recovery behaviors (spin, backup) activate
3. Confirm the robot can recover and continue navigation

## Monitoring and Visualization

### Step 16: Using RViz2 for Navigation

Launch RViz2 with the navigation configuration:

```bash
# Launch RViz2 for navigation monitoring
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

In RViz2, you can:
- Visualize the global and local costmaps
- See the planned path
- Monitor robot pose and trajectory
- Send navigation goals interactively
- View sensor data and SLAM results

### Step 17: Performance Monitoring

Monitor navigation performance with these commands:

```bash
# Monitor navigation performance
ros2 topic hz /navigate_to_pose/_action/status
ros2 topic hz /local_costmap/costmap_updates
ros2 topic hz /global_costmap/costmap_updates

# Check CPU/GPU usage
nvidia-smi
htop
```

## Troubleshooting Common Issues

### Issue 1: Navigation Fails to Start
**Symptoms**: Navigation stack doesn't initialize properly
**Solutions**:
- Verify all required parameters are set
- Check TF tree for proper transforms
- Ensure sensors are publishing data
- Confirm coordinate frames are aligned

### Issue 2: Path Planning Fails
**Symptoms**: Robot cannot find a path to the goal
**Solutions**:
- Check that the goal is in a known area
- Verify costmap is properly updated
- Ensure the goal is not in an obstacle
- Check that the map is available and current

### Issue 3: Local Planner Issues
**Symptoms**: Robot oscillates or cannot follow the path
**Solutions**:
- Adjust controller parameters
- Check velocity limits
- Verify sensor data quality
- Tune obstacle inflation parameters

### Issue 4: SLAM-Nav2 Integration Issues
**Symptoms**: Navigation doesn't work with SLAM maps
**Solutions**:
- Ensure proper coordinate frame alignment
- Check that SLAM and Nav2 use compatible map formats
- Verify TF publishing frequency
- Confirm timing synchronization

## Evaluation Metrics

### Step 18: Navigation Performance Metrics

Evaluate navigation performance using these metrics:

1. **Success Rate**: Percentage of goals reached successfully
2. **Time to Goal**: Time taken to reach the destination
3. **Path Length**: Actual path length vs. optimal path
4. **Obstacle Avoidance**: How effectively obstacles are avoided
5. **Recovery Frequency**: How often recovery behaviors are needed
6. **Localization Accuracy**: How well the robot knows its position

## Summary

In this lab, you've successfully integrated Navigation2 with Isaac Sim and Isaac ROS Visual SLAM to create a complete autonomous navigation system. You've learned how to configure path planners, controllers, and recovery behaviors, and tested navigation in simulated environments. This system provides the foundation for autonomous robot navigation with real-time mapping and obstacle avoidance capabilities.

## Next Steps

In Lab 5, you'll learn how to generate synthetic images for AI model training using Isaac Sim's photorealistic rendering capabilities and domain randomization techniques.