from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="remote_commands",
            executable="ps3",
            name="ps3",
            output="screen",
            # arguments=["/dev/input/eventX"],  # optionally pin a specific device
        ),
    ])
