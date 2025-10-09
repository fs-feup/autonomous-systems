# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="slam",
                executable="slam",
                name="slam",
                output="screen",
                arguments=["--ros-args", "--log-level", "slam:=debug"],
            ),
        ]
    )
