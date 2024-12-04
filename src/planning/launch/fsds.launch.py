# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = "/home/ws/src/planning/config_files/fsds.yaml"

    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="planning",
                name="planning_adapter",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "planning:=info"],
            ),
        ]
    )
