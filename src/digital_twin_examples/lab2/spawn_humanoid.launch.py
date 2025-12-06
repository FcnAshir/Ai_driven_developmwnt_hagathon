from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package and file paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_gazebo_humanoid = get_package_share_directory('gazebo_humanoid_robot')

    # URDF file path
    urdf_file = os.path.join(pkg_gazebo_humanoid, 'urdf', 'humanoid_gazebo.urdf.xacro')

    # Gazebo launch arguments
    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf', '--gui-config', 'default'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read()
        }]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Controller manager
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', 'right_arm_controller', 'leg_controller'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Set "true" to run Gazebo GUI in verbose mode'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Set "true" to run Gazebo in headless mode'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ])