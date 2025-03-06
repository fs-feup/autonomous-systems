from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    evaluator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("evaluator"), "launch", "evaluator.launch.py"]
            )
        ),
    )
    mocker_node_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mocker_node"), "launch", "mocker_node.launch.py"]
            )
        ),
    )
    planning_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning"), "launch", "planning.launch.py"]
            )
        ),
    )
    control_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "control.launch.py"]
            )
        ),
    )
    return LaunchDescription(
        [
            evaluator_launch_description,
            #mocker_node_launch_description,
            planning_launch_description,
            control_launch_description,
        ],
    )
