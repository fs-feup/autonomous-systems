from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[{
                'perception_radius': 5.0,
            }],
            output='screen'
        ),
    ])