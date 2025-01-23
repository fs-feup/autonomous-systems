# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ekf_state_est",
                executable="ekf_state_est",
                name="ekf_state_est",
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
