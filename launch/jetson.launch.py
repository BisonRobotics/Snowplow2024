from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetson_pkg',
            executable='apriltag',
            name='apriltag'
        )
    ])
