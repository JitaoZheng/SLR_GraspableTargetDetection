from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detect_graspable_points',
            executable='publish_pointcloud2',
            name='publish_pointcloud2',
            output='screen'
        )
    ])
