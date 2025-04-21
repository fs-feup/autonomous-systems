from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam"), "launch", "slam.launch.py"])
        ),
    )
    evaluator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("evaluator"), "launch", "evaluator.launch.py"]
            )
        ),
    )
    pacsim_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pacsim"), "launch", "autocross.launch.py"]
            )
        ),
    )
    return LaunchDescription(
        [
            slam_launch_description,
            pacsim_launch_description,
            evaluator_launch_description,
        ],
    )
