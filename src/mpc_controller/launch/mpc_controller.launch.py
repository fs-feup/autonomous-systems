from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mpc_controller",
                executable="mpc_controller_node",
                name="mpc_controller",
                output="screen",
                parameters=[],
            )
        ]
    )
