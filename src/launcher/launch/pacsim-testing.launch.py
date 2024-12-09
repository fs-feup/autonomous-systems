from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_simulated_se = "True"
    use_simulated_perception = "True"
    use_simulated_planning = "False"

    se_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ekf_state_est"), "launch", "pacsim.launch.py"]
            )
        ),
        launch_arguments={
            "use_simulated_perception": use_simulated_perception,
        }.items(),
    )

    evaluator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("evaluator"), "launch", "pacsim.launch.py"]
            )
        ),
        launch_arguments={
            "use_simulated_perception": use_simulated_perception,
            "use_simulated_se": use_simulated_se,
            "use_simulated_planning": use_simulated_planning,
        }.items(),
    )

    planning_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning"), "launch", "pacsim.launch.py"]
            )
        ),
        launch_arguments={
            "use_simulated_se": use_simulated_se,
        }.items(),
    )

    control_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "pacsim.launch.py"]
            )
        ),
        launch_arguments={
            "use_simulated_se": use_simulated_se,
            "use_simulated_planning": use_simulated_planning,
        }.items(),
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
            evaluator_launch_description,
            se_launch_description,
            planning_launch_description,
            control_launch_description,
            mocker_node_launch_description,
        ]
    )
