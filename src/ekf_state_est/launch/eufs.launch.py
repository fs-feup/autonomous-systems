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
                "data_association_gate1",
                description="Data association gate 1, used for setting a threshold of acceptance for associating landmarks",
                default_value="5.991",
            ),  # meters
            DeclareLaunchArgument(
                "data_association_gate2",
                description="Data association gate 2, used for setting a base limit creating new landmark",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "data_association_limit_distance",
                description="Data association limit distance for considering a landmark as worth analyzing as new or existing landmark",
                default_value="15",
            ),
            DeclareLaunchArgument(
                "IMU_noise",
                description="Noise value for IMU",
                default_value="0.00000064",
            ),
            DeclareLaunchArgument(
                "data_association_model",
                description="Data Association Model Name",
                default_value="max_likelihood",
            ),
            DeclareLaunchArgument(
                "observation_noise",
                description="Noise value for observations",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "wheel_speed_sensor_noise",
                description="Noise value for wheel speed sensors",
                default_value="0.00002",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
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
                    {
                        "data_association_model": LaunchConfiguration(
                            "data_association_model"
                        )
                    },
                    {
                        "data_association_limit_distance": LaunchConfiguration(
                            "data_association_limit_distance"
                        )
                    },
                    {"adapter": LaunchConfiguration("adapter")},
                    {"IMU_noise": LaunchConfiguration("IMU_noise")},
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
                    {
                        "data_association_gate1": LaunchConfiguration(
                            "data_association_gate1"
                        )
                    },
                    {
                        "data_association_gate2": LaunchConfiguration(
                            "data_association_gate2"
                        )
                    },
                ],
                arguments=["--ros-args", "--log-level", "ekf_state_est:=info"],
            ),
        ]
    )
