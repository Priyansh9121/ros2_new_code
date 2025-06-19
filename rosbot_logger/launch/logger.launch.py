from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot_logger',
            executable='logger_node',
            name='logger_node',
            output='screen',
            parameters=[{'log_to_file': True}]
        )
    ])
