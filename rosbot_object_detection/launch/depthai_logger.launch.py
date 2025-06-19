#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ───────── launch-time arguments ─────────
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.4',
        description='Minimum ORB match confidence needed to publish a detection'
    )

    # ───────── detector node ─────────
    detector_node = Node(
        package='rosbot_object_detection',
        executable='depthai_logger_node',     # console-script from setup.py
        name='depthai_logger_node',
        output='screen',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold')
        }]
    )

    # ───────── launch description ─────────
    return LaunchDescription([
        confidence_arg,
        detector_node
    ])
