# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="planning",
                name="planning_adapter",
                arguments=["--ros-args", "--log-level", "planning:=info"],
            ),
        ]
    )
