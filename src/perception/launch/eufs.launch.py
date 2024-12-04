# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = "/home/ws/src/perception/config_files/eufs.yaml"

    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="perception",
                name="perception_adapter",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "perception:=info"],
            ),
        ]
    )
