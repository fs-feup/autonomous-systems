# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="velcity_estimation",
                executable="velcity_estimation",
                name="velcity_estimation",
                arguments=["--ros-args", "--log-level", "velcity_estimation:=info"],
            ),
        ]
    )