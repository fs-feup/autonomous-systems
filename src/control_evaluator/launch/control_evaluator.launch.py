from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="control_evaluator",
                executable="control_evaluator",
                name="control_evaluator",
                output="screen",
            )
        ]
    )
