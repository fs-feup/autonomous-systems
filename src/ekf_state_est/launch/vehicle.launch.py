# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = "/home/ws/src/ekf_state_est/config_files/vehicle.yaml"

    return LaunchDescription(
        [
            Node(
                package="ekf_state_est",
                executable="ekf_state_est",
                name="ekf_state_est",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
