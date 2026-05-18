from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("perception"), "launch", "perception.launch.py"]
            )
        ),
    )
    slam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam"), "launch", "slam.launch.py"]
            )
        ),
    )
    
    ve_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("velocity_estimation"),
                    "launch",
                    "velocity_estimation.launch.py",
                ]
            )
        ),
    )
    return LaunchDescription(
        [
            perception_launch_description,
            slam_launch_description,
            ve_launch_description,
        ],
    )
