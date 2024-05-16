from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                description="Vehicle or Simulation mode (pacsim, fsds, eufs)",
                default_value="vehicle",
            ),
            Node(
                package="control",
                executable="control",
                name="control",
                parameters=[
                    {"mode": LaunchConfiguration("mode")},
                ],
                arguments=['--ros-args', '--log-level', 'control:=debug'],
            ),
        ]
    )
