# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_file = "/home/ws/src/control/config_files/pacsim.yaml"

    return LaunchDescription(
        [
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[{"adapter": "pacsim"}, config_file],
                arguments=["--ros-args", "--log-level", "control:=info"],
            ),
        ]
    )
