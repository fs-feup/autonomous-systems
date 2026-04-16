from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    discipline_arg = DeclareLaunchArgument(
        "discipline",
        default_value="autocross",
        description="Competition discipline used by planning and invictasim.",
    )

    env = [
        SetEnvironmentVariable(name="AS_ADAPTER", value="invictasim"),
        SetEnvironmentVariable(name="AS_USE_SIMULATED_SE", value="true"),
        SetEnvironmentVariable(name="AS_USE_SIMULATED_VELOCITIES", value="true"),
        SetEnvironmentVariable(name="AS_USE_SIMULATED_PLANNING", value="false"),
        SetEnvironmentVariable(name="AS_DISCIPLINE", value=LaunchConfiguration("discipline")),
        SetEnvironmentVariable(name="INVICTASIM_INPUT_ADAPTER", value="ros"),
        SetEnvironmentVariable(name="INVICTASIM_OUTPUT_ADAPTER", value="ros"),
    ]

    invictasim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("invictasim"), "launch", "invictasim.launch.py"]
            )
        ),
    )
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("planning"), "launch", "planning.launch.py"]
            )
        ),
    )
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("control"), "launch", "control.launch.py"]
            )
        ),
    )

    return LaunchDescription(
        [
            discipline_arg,
            *env,
            invictasim_launch,
            planning_launch,
            control_launch,
        ]
    )
