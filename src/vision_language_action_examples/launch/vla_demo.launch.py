from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='simple',
        description='Demo mode: simple, advanced, or interactive'
    )

    # VLA Demo Node
    vla_demo_node = Node(
        package='vision_language_action_examples',
        executable='simple_vla_demo',
        name='vla_demo_node',
        parameters=[
            {'demo_mode': LaunchConfiguration('demo_mode')}
        ],
        output='screen'
    )

    # Optional: RViz for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('vision_language_action_examples'),
        'rviz',
        'vla_demo.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='vla_rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        demo_mode_arg,
        vla_demo_node,
        # Commenting out rviz for now as config file may not exist
        # rviz_node
    ])