import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'llm_robot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include action files
        (os.path.join('share', package_name, 'action'),
         ['action/RobotCommand.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='LLM agent for robot command interpretation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_executor_server = llm_robot_agent.robot_executor_server:main',
            'llm_client = llm_robot_agent.llm_client:main',
            'llm_interface = llm_robot_agent.llm_interface:main',
        ],
    },
)