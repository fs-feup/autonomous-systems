# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mocker_node",
                executable="mocker_node",
                name="mocker_node",
                arguments=["--ros-args", "--log-level", "mocker_node:=info"],
            ),
        ]
    )
