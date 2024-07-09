# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "max_angle",
                description="max angle for the turning of the inspection script",
                default_value="0.392699",
            ),  # Pi / 6.0, rad; 22.5 degrees in rad
            DeclareLaunchArgument(
                "inspection_ideal_velocity", default_value="1.0"
            ),  # m/s
            DeclareLaunchArgument(
                "ebs_test_ideal_velocity", default_value="2.0"
            ),  # m/s
            DeclareLaunchArgument(
                "turning_period",
                description="Time to perform a full turning cycle in inspection",
                default_value="4.0",
            ),  # seconds
            DeclareLaunchArgument("wheel_radius", default_value="0.254"),
            DeclareLaunchArgument(
                "inspection_gain",
                description="Gains for longitudinal P controllers of inspection",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "ebs_test_gain",
                description="Gains for longitudinal P controllers of inspection",
                default_value="0.25",
            ),
            DeclareLaunchArgument("finish_time", default_value="26.0"),  # seconds
            DeclareLaunchArgument(
                "start_and_stop",
                description="Normal mode or testing regenerative braking mode",
                default_value="False",
            ),
            Node(
                package="inspection",
                executable="inspection",
                name="inspection",
                parameters=[
                    {"turning_period": LaunchConfiguration("turning_period")},
                    {"max_angle": LaunchConfiguration("max_angle")},
                    {
                        "inspection_ideal_velocity": LaunchConfiguration(
                            "inspection_ideal_velocity"
                        )
                    },
                    {
                        "ebs_test_ideal_velocity": LaunchConfiguration(
                            "ebs_test_ideal_velocity"
                        )
                    },
                    {"wheel_radius": LaunchConfiguration("wheel_radius")},
                    {"inspection_gain": LaunchConfiguration("inspection_gain")},
                    {"ebs_test_gain": LaunchConfiguration("ebs_test_gain")},
                    {"finish_time": LaunchConfiguration("finish_time")},
                    {"start_and_stop": LaunchConfiguration("start_and_stop")},
                ],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=debug"],
            ),
        ]
    )
