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
                "angle_gain",
                description="Gain for the angle in the cone coloring cost function",
                default_value="3.7",
            ),
            DeclareLaunchArgument(
                "distance_gain",
                description="Gain for the distance in the cone coloring cost function",
                default_value="8.0",
            ),
            DeclareLaunchArgument(
                "ncones_gain",
                description="Gain for the number of cones in the cone coloring cost funtcion",
                default_value="8.7",
            ),
            DeclareLaunchArgument(
                "angle_exponent",
                description="Exponent on the angle in the cone coloring cost function",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "distance_exponent",
                description="Exponent on the distance in the cone coloring cost function",
                default_value="1.7",
            ),
            DeclareLaunchArgument(
                "cost_max",
                description="Maximum allowed cost to place a cone in the cone coloring cost function",
                default_value="40.0",
            ),
            DeclareLaunchArgument(
                "outliers_spline_order",
                description="Order of the spline to remove outliers",
                default_value="3",
            ),
            DeclareLaunchArgument(
                "outliers_spline_coeffs_ratio",
                description="Ratio of coefficients to cone count to fit spline when dealing with outliers",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "outliers_spline_precision",
                description="Ratio of cones in the spline to initial number of cones when dealing with outliers",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "smoothing_spline_order",
                description="Order of the spline to remove smooth the path",
                default_value="3",
            ),
            DeclareLaunchArgument(
                "smoothing_spline_coeffs_ratio",
                description="Ratio of coefficients to cone count to fit spline when smoothing the path",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "smoothing_spline_precision",
                description="Ratio of cones in the spline to initial number of cones when smoothing the path",
                default_value="10",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Whether to use simulated state estimation",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "publishing_visualization_msg",
                description="Whether to publish path in visualization format",
                default_value="1",
            ),
            Node(
                package="planning",
                executable="planning",
                name="planning",
                parameters=[
                    {"angle_gain": LaunchConfiguration("angle_gain")},
                    {"distance_gain": LaunchConfiguration("distance_gain")},
                    {"ncones_gain": LaunchConfiguration("ncones_gain")},
                    {"angle_exponent": LaunchConfiguration("angle_exponent")},
                    {"distance_exponent": LaunchConfiguration("distance_exponent")},
                    {"cost_max": LaunchConfiguration("cost_max")},
                    {
                        "outliers_spline_order": LaunchConfiguration(
                            "outliers_spline_order"
                        )
                    },
                    {
                        "outliers_spline_coeffs_ratio": LaunchConfiguration(
                            "outliers_spline_coeffs_ratio"
                        )
                    },
                    {
                        "outliers_spline_precision": LaunchConfiguration(
                            "outliers_spline_precision"
                        )
                    },
                    {
                        "smoothing_spline_order": LaunchConfiguration(
                            "smoothing_spline_order"
                        )
                    },
                    {
                        "smoothing_spline_coeffs_ratio": LaunchConfiguration(
                            "smoothing_spline_coeffs_ratio"
                        )
                    },
                    {
                        "smoothing_spline_precision": LaunchConfiguration(
                            "smoothing_spline_precision"
                        )
                    },
                    {"adapter": LaunchConfiguration("adapter")},
                    {"use_simulated_se": LaunchConfiguration("use_simulated_se")},
                ],
                arguments=["--ros-args", "--log-level", "planning:=info"],
            ),
        ]
    )
