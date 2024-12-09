# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = "/home/ws/src/evaluator/config_files/fsds.yaml"

    return LaunchDescription(
        [
            Node(
                package="evaluator",
                executable="evaluator",
                name="evaluator",
                parameters=[config_file],
            ),
        ]
    )
