# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_file = "/home/ws/src/control/config_files/vehicle.yaml"

    return LaunchDescription(
        [
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[{"adapter": "vehicle"}, config_file],
                arguments=["--ros-args", "--log-level", "control:=debug"],
            ),
        ]
    )
