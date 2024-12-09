# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    config_file = "/home/ws/src/control/config_files/vehicle.yaml"

    final_params = {
        "use_simulated_se": LaunchConfiguration("use_simulated_se"),
        "use_simulated_planning": LaunchConfiguration("use_simulated_planning"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Use Simulated State Estimation",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_simulated_planning",
                description="Use Simulated Planning",
                default_value="false",
            ),
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[{"adapter": "vehicle"}, config_file, final_params],
                arguments=["--ros-args", "--log-level", "control:=debug"],
            ),
        ]
    )
