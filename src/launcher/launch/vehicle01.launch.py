from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    se_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ekf_state_est"), "launch", "vehicle.launch.py"]
            )
        ),
    )
    perception_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("perception"), "launch", "vehicle.launch.py"]
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
            se_launch_description,
            perception_launch_description,
            planning_launch_description,
            control_launch_description,
        ],
    )
