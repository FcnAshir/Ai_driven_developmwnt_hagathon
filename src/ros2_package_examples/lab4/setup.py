import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'joint_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Joint controller for humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = joint_controller_pkg.joint_controller:main',
            'advanced_joint_controller = joint_controller_pkg.advanced_joint_controller:main',
        ],
    },
)