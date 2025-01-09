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
                package="evaluator",
                executable="evaluator",
                name="evaluator",
                arguments=["--ros-args", "--log-level", "evaluator:=info"],
            ),
        ]
    )
