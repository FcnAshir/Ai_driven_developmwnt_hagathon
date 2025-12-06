from setuptools import setup

package_name = 'isaac_examples'

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
    description='Isaac Sim examples for humanoid robotics simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_basic_setup = isaac_examples.isaac_basic_setup:main',
            'isaac_robot_control = isaac_examples.isaac_robot_control:main',
            'isaac_sensor_simulation = isaac_examples.isaac_sensor_simulation:main',
            'isaac_terrain_generation = isaac_examples.isaac_terrain_generation:main',
            'isaac_ai_integration = isaac_examples.isaac_ai_integration:main',
            'isaac_advanced_scenarios = isaac_examples.isaac_advanced_scenarios:main',
        ],
    },
)