# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="velocity_estimation",
                executable="velocity_estimation",
                name="velocity_estimation",
                arguments=["--ros-args", "--log-level", "velocity_estimation:=info"],
            ),
        ]
    )