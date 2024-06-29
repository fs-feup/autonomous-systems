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
                "track_name",
                description="name of the track(folder) where the track map and ground truths for \
                state estimation and planning are stored",
                default_value="small_track",
            ),
            DeclareLaunchArgument(
                "sim",
                description="name of the simulator where the track we pretend belongs to",
                default_value="eufs",
            ),
            Node(
                package="mocker_node",
                executable="mocker_node",
                name="mocker_node",
                parameters=[
                    {"track_name": LaunchConfiguration("track_name")},
                    {"sim": LaunchConfiguration("sim")},
                ],
            ),
        ]
    )
