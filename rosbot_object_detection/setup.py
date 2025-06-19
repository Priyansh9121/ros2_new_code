#!/usr/bin/env python3
# setup.py — rosbot_object_detection  (ROS 2 Humble / ament_python)

from setuptools import setup, find_packages
from glob import glob
import os

package_name = "rosbot_object_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),                       # rosbot_object_detection/
    package_data={package_name: ["templates/*"]},   # install template images
    data_files=[
        #  install the marker code
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),

        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS User',
    maintainer_email='user@example.com',
    description='ORB-based object detection using OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depthai_logger_node = rosbot_object_detection.depthai_logger_node:main',
        ],
    },
)

