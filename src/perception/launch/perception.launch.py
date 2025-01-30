# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="perception",
                name="perception_adapter",
                arguments=["--ros-args", "--log-level", "perception:=info"],
            ),
        ]
    )