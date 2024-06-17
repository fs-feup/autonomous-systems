from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mocker_node",
                description="Wether or not to use Mocker Node for Plannning (true/false)",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Vehicle or Simulation mode (pacsim, fsds, eufs)",
                default_value="pacsim",
            ),
            DeclareLaunchArgument(
                "lookahead_gain",
                description="Variable K -> Lookahead Gain",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Use Simulated State Estimation, that is, vehicle state from simulator (true/false)",
                default_value="true",
            ),
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[
                    {"adapter": LaunchConfiguration("adapter")},
                    {"mocker_node": LaunchConfiguration("mocker_node")},
                    {"lookahead_gain": LaunchConfiguration("lookahead_gain")},
                    {"use_simulated_se": LaunchConfiguration("use_simulated_se")},
                ],
                arguments=["--ros-args", "--log-level", "control:=debug"],
            ),
            
        ]
    )