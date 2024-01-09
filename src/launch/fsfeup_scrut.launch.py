from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package='loc_map',
            executable='loc_map',
            output='screen'
        ),
        Node(
            package='control',
            executable='control',
            output='screen'
        ),
        Node(
            package='can',
            executable='can',
            output='screen'
        )
    ]

    return LaunchDescription(nodes)