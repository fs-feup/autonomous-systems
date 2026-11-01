"""Run InvictaSim with VE, SLAM, planning, and control in closed loop."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            _include("invictasim", "invictasim.launch.py"),
            _include("velocity_estimation", "velocity_estimation.launch.py"),
            _include("slam", "slam.launch.py"),
            _include("planning", "planning.launch.py"),
            _include("control", "control.launch.py"),
        ]
    )
