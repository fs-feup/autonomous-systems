from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    inspection_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("inspection"), "launch", "normal.launch.py"]
            )
        ),
        launch_arguments={
            "max_angle": "0.52359877559",
            "inspection_ideal_velocity": "1.0",
            "ebs_test_ideal_velocity": "2.0",
            "turning_period": "4.0",
            "wheel_radius": "0.254",
            "inspection_gain": "0.25",
            "ebs_test_gain": "0.25",
            "finish_time": "26.0",
            "start_and_stop": "False",
        }.items(),
    )
    return LaunchDescription(
        [inspection_launch_description],
    )
