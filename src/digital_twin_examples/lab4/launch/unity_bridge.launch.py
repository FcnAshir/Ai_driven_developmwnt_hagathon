from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('unity_ros_bridge'),
            'worlds',
            'unity_export_world.sdf'
        ]),
        description='SDF world file'
    )

    # Unity Bridge Node
    unity_bridge_node = Node(
        package='unity_ros_bridge',
        executable='unity_bridge_node',
        name='unity_bridge_node',
        parameters=[
            {'unity_host': 'localhost'},
            {'unity_port': 10000}
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description':
                PathJoinSubstitution([
                    FindPackageShare('unity_ros_bridge'),
                    'urdf',
                    'humanoid_robot.urdf'
                ])
            }
        ]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'rate': 50}
        ]
    )

    # Gazebo World Launch (if available)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    return LaunchDescription([
        world_arg,
        unity_bridge_node,
        robot_state_publisher,
        joint_state_publisher,
        # Commenting out gazebo launch as it may not be available in all environments
        # gazebo_launch
    ])