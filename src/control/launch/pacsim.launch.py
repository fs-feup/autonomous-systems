# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = "/home/ws/src/control/config_files/pacsim.yaml"

    final_params = {
        "adapter": LaunchConfiguration("adapter"),
        "use_simulated_se": LaunchConfiguration("use_simulated_se"),
        "use_simulated_planning": LaunchConfiguration("use_simulated_planning"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "adapter",
                description="Vehicle or Simulation mode (e.g., pacsim, vehicle, eufs, fsds)",
                default_value="pacsim",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Use Simulated State Estimation",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_simulated_planning",
                description="Use Simulated Planning",
                default_value="true",
            ),
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[
                    config_file,
                    final_params,
                ],
                arguments=["--ros-args", "--log-level", "control:=info"],
            ),
        ]
    )
