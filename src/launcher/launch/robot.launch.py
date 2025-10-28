from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    factory_robot_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("factory_robot"), "launch", "factory.launch.py"]
            )
        ),
    )
    perception_node_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_perception"), "launch", "perception.launch.py"]
            )
        ),
    )

    return LaunchDescription(
        [
            factory_robot_launch_description,
            perception_node_launch_description,
        ],
    )
