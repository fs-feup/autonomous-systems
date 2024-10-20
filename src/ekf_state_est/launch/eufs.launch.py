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
                default_value="15.00",
            ),  # meters
            DeclareLaunchArgument(
                "wss_noise",
                description=" Noise value for wheel speed sensors (sigma)",
                default_value="0.003",
            ),
            DeclareLaunchArgument(
                "imu_noise",
                description="Noise value for IMU (sigma)",
                default_value="0.0064",
            ),
            DeclareLaunchArgument(
                "observation_noise",
                description="Noise value for observations (sigma)",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "motion_model",
                description="Motion model to use",
                default_value="imu_velocity_model",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
            ),
            DeclareLaunchArgument(
                "data_assocation_model",
                description="data assciation model to use",
                default_value="max_likelihood",
            ),
            DeclareLaunchArgument(
                "use_odometry",
                description="Either use odometry or IMU (TODO: remove for complete velocity estimation)",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "use_simulated_perception",
                description="Use simulated perception from simulator instead of Perception node",
                default_value="False",
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
                    {
                        "data_assocation_model": LaunchConfiguration(
                            "data_assocation_model"
                        )
                    },
                    {"imu_noise": LaunchConfiguration("imu_noise")},
                    {"wss_noise": LaunchConfiguration("wss_noise")},
                    {"use_odometry": LaunchConfiguration("use_odometry")},
                    {
                        "use_simulated_perception": LaunchConfiguration(
                            "use_simulated_perception"
                        )
                    },
                    {"observation_noise": LaunchConfiguration("observation_noise")},
                ],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
