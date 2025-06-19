from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot_obstacle_bypass',
            executable='bypass_node',
            name='bypass_node',
            output='screen',
            parameters=[
                {'obstacle_threshold': 0.5},
                {'base_detour_offset': 1.0},
                {'input_goal_topic':  '/hololens_goal'},
            ]
        )
    ])

