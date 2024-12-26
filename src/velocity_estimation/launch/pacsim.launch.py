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
                "car_model",
                description="Car model to use",
                default_value="bicycle-model",
            ),
            DeclareLaunchArgument(
                "motion_model",
                description="Motion model to use",
                default_value="ctra-no-slip",
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
                parameters=[
                    {"car_model": LaunchConfiguration("car_model")},
                    {"motion_model": LaunchConfiguration("motion_model")},
                    {"adapter": LaunchConfiguration("adapter")}
                ],
                arguments=["--ros-args", "--log-level", "velocity_estimation:=info"],
            ),
        ]
    )
