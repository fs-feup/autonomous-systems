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

    state_estimation_node = Node(
        package='robot_state_estimation',
        executable='state_estimation_node',
        name='state_estimation',
        output='screen'
    )

    planner_node = Node(
        package='robot_path_planning',
        executable='path_planning_node',
        name='path_planning',
        output='screen'
    )

    controller_node = Node(
        package='robot_control',
        executable='control_node',
        name='robot_control',
        output='screen'
    )

    return LaunchDescription(
        [
            factory_robot_launch_description,
            perception_node_launch_description,
            state_estimation_node,
            planner_node,
            controller_node,
        ],
    )
