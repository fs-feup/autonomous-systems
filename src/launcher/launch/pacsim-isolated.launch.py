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
                [FindPackageShare("ekf_state_est"), "launch", "pacsim.launch.py"]
            )
        ),
    )
    evaluator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("evaluator"), "launch", "pacsim.launch.py"]
            )
        ),
    )
    planning_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning"), "launch", "pacsim.launch.py"]
            )
        ),
    )
    control_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "pacsim.launch.py"]
            )
        ),
    )
    mocker_node_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mocker_node"), "launch", "pacsim.launch.py"]
            )
        ),
    )
    return LaunchDescription(
        [
            se_launch_description,
            evaluator_launch_description,
            planning_launch_description,
            control_launch_description,
            mocker_node_launch_description,
        ],
    )
