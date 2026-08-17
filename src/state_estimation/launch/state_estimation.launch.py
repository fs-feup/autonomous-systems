# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="state_estimation",
                executable="state_estimation",
                name="state_estimation",
                arguments=["--ros-args", "--log-level", "state_estimation:=info"],
            ),
        ]
    )
