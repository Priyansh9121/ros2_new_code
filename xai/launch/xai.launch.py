from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # ───── Launch object detection ─────
        Node(
            package="rosbot_object_detection",
            executable="depthai_logger_node",
            name="rosbot_object_detection",
            output="screen"
        ),

        # ───── Launch obstacle bypass ─────
        Node(
            package="rosbot_obstacle_bypass",
            executable="bypass_node",
            name="obstacle_bypass_node",
            output="screen"
        ),

        # ───── Launch event logger ─────
        Node(
            package="rosbot_logger",
            executable="logger_node",
            name="logger_node",
            output="screen"
        ),

        # ───── Reset state after system is up ─────
        TimerAction(
            period=5.0,  # Wait 5 seconds for Nav2 and RViz to come up
            actions=[

                # Clear global costmap
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/global_costmap/clear_entirely_global_costmap',
                        'std_srvs/srv/Empty', '{}'
                    ],
                    output='screen'
                ),

                # Clear local costmap
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/local_costmap/clear_entirely_local_costmap',
                        'std_srvs/srv/Empty', '{}'
                    ],
                    output='screen'
                ),


            ]
        )
    ])

