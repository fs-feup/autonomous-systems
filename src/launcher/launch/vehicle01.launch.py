from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("perception"), "launch", "vehicle.launch.py"]
            )
        ),
    )
    se_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ekf_state_est"), "launch", "vehicle.launch.py"]
            )
        ),
    )
    planning_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning"), "launch", "vehicle.launch.py"]
            )
        ),
    )
    control_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "vehicle.launch.py"]
            )
        ),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Use Simulated State Estimation, that is, vehicle state from simulator (true/false)",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "use_simulated_perception",
                description="Whether the system is using simulated perception or not",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "use_simulated_planning",
                description="Whether the system is using simulated Planning or not",
                default_value="True",
            ),
            perception_launch_description,
            se_launch_description,
            planning_launch_description,
            control_launch_description,
        ],
    )
