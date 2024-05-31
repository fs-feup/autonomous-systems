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
                "data_association_limit_distance",
                description="Maximum distance to admit landmarks",
                default_value="15.0",
            ),  # meters
            DeclareLaunchArgument(
                "sml_initial_limit",
                description="Initial limit for the limit function used in the simple maximum likelihood data association",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "sml_curvature",
                description="Curvature for the limit function used in the simple maximum likelihood data association",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "observation_noise",
                description="Noise value for observations (sigma)",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "wheel_speed_sensor_noise",
                description="Noise value for wheel speed sensors (sigma)",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "motion_model",
                description="Motion model to use",
                default_value="normal_velocity_model",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
            ),
            DeclareLaunchArgument(
                "use_odometry",
                description="Either use odometry or IMU (TODO: remove for complete velocity estimation)",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "use_simulated_perception",
                description="Use simulated perception from simulator instead of Perception node",
                default_value="True",
            ),
            Node(
                package="ekf_state_est",
                executable="ekf_state_est",
                name="ekf_state_est",
                parameters=[
                    {"motion_model": LaunchConfiguration("motion_model")},
                    {
                        "data_association_limit_distance": LaunchConfiguration(
                            "data_association_limit_distance"
                        )
                    },
                    {"adapter": LaunchConfiguration("adapter")},
                    {"use_odometry": LaunchConfiguration("use_odometry")},
                    {
                        "use_simulated_perception": LaunchConfiguration(
                            "use_simulated_perception"
                        )
                    },
                    {"observation_noise": LaunchConfiguration("observation_noise")},
                    {
                        "wheel_speed_sensor_noise": LaunchConfiguration(
                            "wheel_speed_sensor_noise"
                        )
                    },
                    {"sml_curvature": LaunchConfiguration("sml_curvature")},
                    {"sml_initial_limit": LaunchConfiguration("sml_initial_limit")},
                ],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
