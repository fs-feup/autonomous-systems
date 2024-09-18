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
                default_value="9.246",
            ),
            DeclareLaunchArgument(
                "distance_gain",
                description="Gain for the distance in the cone coloring cost function",
                default_value="6.657",
            ),
            DeclareLaunchArgument(
                "ncones_gain",
                description="Gain for the number of cones in the cone coloring cost funtcion",
                default_value="20.7",
            ),
            DeclareLaunchArgument(
                "angle_exponent",
                description="Exponent on the angle in the cone coloring cost function",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "distance_exponent",
                description="Exponent on the distance in the cone coloring cost function",
                default_value="0.698",
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
                "path_calculation_dist_threshold",
                description="Max distance limit for a Delaunay Triangulation to be valid",
                default_value="7.0",
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
                default_value="pacsim",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Whether to use simulated state estimation",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "publishing_visualization_msg",
                description="Whether to publish path in visualization format",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "pre_defined_velocity_planning",
                description="A pre-defined velocity planning value",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "use_outlier_removal",
                description="Whether to use outlier removal or to skip it",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_path_smoothing",
                description="Whether to use path smoothing or to skip it",
                default_value="true",
            ),
            Node(
                package="planning",
                executable="planning",
                name="planning_adapter",
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
                        "path_calculation_dist_threshold": LaunchConfiguration(
                            "path_calculation_dist_threshold"
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
                    {
                        "publishing_visualization_msg": LaunchConfiguration(
                            "publishing_visualization_msg"
                        )
                    },
                    {
                        "pre_defined_velocity_planning": LaunchConfiguration(
                            "pre_defined_velocity_planning"
                        )
                    },
                    {"use_outlier_removal": LaunchConfiguration("use_outlier_removal")},
                    {"use_path_smoothing": LaunchConfiguration("use_path_smoothing")},
                ],
                arguments=["--ros-args", "--log-level", "planning:=info"],
            ),
        ]
    )
