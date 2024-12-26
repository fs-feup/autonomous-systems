# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "estimation_method",
                description="Estimation method to use",
                default_value="ekf",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="pacsim",
            ),
            Node(
                package="velocity_estimation",
                executable="velocity_estimation",
                name="temporary_ve_adapter",
                output="screen",
                parameters=[
                    {"estimation_method": LaunchConfiguration("estimation_method")},
                    {"adapter": LaunchConfiguration("adapter")}
                ],
                arguments=["--ros-args", "--log-level", "velocity_estimation:=info"],
            ),
        ]
    )
