from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "adapter",
                description="Vehicle or Simulation mode (pacsim, fsds, eufs)",
                default_value="vehicle",
            ),
            DeclareLaunchArgument(
                "mocker_node",
                description="Wether or not to use Mocker Node for Plannning (true/false)",
                default_value="false",
            ),
            Node(
                package="control",
                executable="node_control",
                name="control",
                parameters=[
                    {"adapter": LaunchConfiguration("adapter")},
                    {"mocker_node": LaunchConfiguration("mocker_node")}
                ],
                arguments=['--ros-args', '--log-level', 'control:=debug'],
            ),
        ]
    )
