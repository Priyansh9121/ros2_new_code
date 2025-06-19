from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'xai'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Will detect xai/
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/xai']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group25',
    maintainer_email='group25@student.rmit.edu.au',
    description='Combined launch package for rosbot subsystems',
    license='RMIT',
    entry_points={},
)

