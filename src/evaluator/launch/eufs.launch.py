# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = "/home/ws/src/evaluator/config_files/eufs.yaml"

    final_params = {
        "use_simulated_perception": LaunchConfiguration("use_simulated_perception"),
        "use_simulated_se": LaunchConfiguration("use_simulated_se"),
        "use_simulated_planning": LaunchConfiguration("use_simulated_planning"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_perception",
                description="Use Simulated Perception",
                default_value="true",
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
                package="evaluator",
                executable="evaluator",
                name="evaluator",
                parameters=[config_file, final_params],
                arguments=["--ros-args", "--log-level", "evaluator:=info"],
            ),
        ]
    )
