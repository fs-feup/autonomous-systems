# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = "/home/ws/src/ekf_state_est/config_files/eufs.yaml"

    final_params = {
        "use_simulated_perception": LaunchConfiguration("use_simulated_perception"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_perception",
                description="Use Simulated Perception",
                default_value="False",
            ),
            Node(
                package="ekf_state_est",
                executable="ekf_state_est",
                name="ekf_state_est",
                parameters=[config_file, final_params],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
