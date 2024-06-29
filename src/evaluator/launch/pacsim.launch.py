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
                "use_simulated_perception",
                description="Whether the system is using simulated perception or not",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Whether the system is using simulated State Estimation or not",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "use_simulated_planning",
                description="Whether the system is using simulated Planning or not",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Which simulation environment to use",
                default_value="pacsim",
            ),
            Node(
                package="evaluator",
                executable="evaluator",
                name="evaluator",
                parameters=[
                    {"adapter": LaunchConfiguration("adapter")},
                    {
                        "use_simulated_perception": LaunchConfiguration(
                            "use_simulated_perception"
                        )
                    },
                    {"use_simulated_se": LaunchConfiguration("use_simulated_se")},
                    {
                        "use_simulated_planning": LaunchConfiguration(
                            "use_simulated_planning"
                        )
                    },
                ],
                arguments=["--ros-args", "--log-level", "evaluator:=info"],
            ),
        ]
    )
