from setuptools import setup

package_name = 'vla_agents'

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
    description='Vision-Language-Action agents for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_command = vla_agents.whisper_command:main',
            'llm_action_planner = vla_agents.llm_action_planner:main',
            'simulated_object_detector = vla_agents.simulated_object_detector:main',
            'robot_pick_object = vla_agents.robot_pick_object:main',
            'clean_room_vla = vla_agents.clean_room_vla:main',
        ],
    },
)