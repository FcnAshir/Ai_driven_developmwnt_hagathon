from setuptools import setup

package_name = 'ros2_package_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package examples for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = ros2_package_examples.simple_node:main',
            'talker_node = ros2_package_examples.talker_node:main',
            'listener_node = ros2_package_examples.listener_node:main',
            'joint_controller = ros2_package_examples.joint_controller:main',
            'advanced_joint_controller = ros2_package_examples.advanced_joint_controller:main',
            'robot_executor_server = ros2_package_examples.robot_executor_server:main',
            'llm_client = ros2_package_examples.llm_client:main',
            'llm_interface = ros2_package_examples.llm_interface:main',
        ],
    },
)