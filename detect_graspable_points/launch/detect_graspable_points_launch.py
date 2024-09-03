from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detect_graspable_points',
            executable='detect_graspable_points_main',
            name='detect_graspable_points_main',
            output='screen'
        )
    ])
