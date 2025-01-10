# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                arguments=["--ros-args", "--log-level", "control:=info"],
            ),
        ]
    )