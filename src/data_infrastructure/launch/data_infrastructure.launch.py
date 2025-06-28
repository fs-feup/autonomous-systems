# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="data_infrastructure",
                executable="data_infrastructure_node",
                name="data_infrastructure",
                arguments=['--ros-args', '--log-level', 'data_infrastructure_node:=info']
            ),
        ]
    )
