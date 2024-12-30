from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_frame_id",
                description="vehicle's frame id",
                default_value="hesai_lidar",
            ),
            DeclareLaunchArgument(
                "horizontal_resolution",
                description="Lidar's horizontal resolution",
                default_value="0.33",
            ),
            DeclareLaunchArgument(
                "vertical_resolution",
                description="Lidar's vertical resolution",
                default_value="0.2",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="vehicle",
            ),
            DeclareLaunchArgument(
                "fov_trim_angle",
                description="Trim the points received to a max angle",
                default_value="45",  # degrees
            ),
            DeclareLaunchArgument(
                "pc_max_range",
                description="Point cloud maximum distance filtering (m)",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "pc_min_range",
                description="Point cloud minimum distance filtering (m)",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "pc_rlidar_max_height",
                description="Point cloud height filter (relative to LIDAR) (m)",
                default_value="-0.22",
            ),
            DeclareLaunchArgument(
                "ground_removal",
                description="Ground Removal algorithm",
                default_value="grid_ransac",
            ),
            DeclareLaunchArgument(
                "ransac_epsilon",
                description="RANSAC epsilon threshold",
                default_value="0.09",
            ),
            DeclareLaunchArgument(
                "ransac_iterations",
                description="RANSAC number of iterations",
                default_value="40",
            ),
            DeclareLaunchArgument(
                "n_angular_grids",
                description="Number of angular grids",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "radius_resolution",
                description="Radius size of a radius grid (m)",
                default_value="7.5",
            ),
            DeclareLaunchArgument(
                "clustering_n_neighbours",
                description="Number of neighbours for Clustering algorithm",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "clustering_epsilon",
                description="Epsilon for Clustering algorithm",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "target_file",
                description="PCD file where the target cone is saved",
                default_value="cone.pcd",
            ),
            DeclareLaunchArgument(
                "max_correspondence_distance",
                description="Maximum correspondence distance for ICP",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "max_iteration",
                description="Maximum number of iterations for ICP",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "transformation_epsilon",
                description="Transformation epsilon for convergence criteria",
                default_value="1e-8",
            ),
            DeclareLaunchArgument(
                "euclidean_fitness_epsilon",
                description="Euclidean fitness epsilon for convergence criteria",
                default_value="1e-6",
            ),
            DeclareLaunchArgument(
                "min_height",
                description="Minimum height of a cluster to be considered a cone",
                default_value="0.13",
            ),
            DeclareLaunchArgument(
                "large_max_height",
                description="Maximum height of a cluster to be considered a large cone",
                default_value="0.57",  # not on ground tested
            ),
            DeclareLaunchArgument(
                "small_max_height",
                description="Maximum height of a cluster to be considered a small cone",
                default_value="0.36",
            ),
            DeclareLaunchArgument(
                "out_distance_cap",
                description="Minimum out distance value cap for it to be 0 for a point",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "min_xoy",
                description="Minimum xOy plane deviation",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "max_xoy",
                description="Maximum xOy plane deviation",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "min_z", description="Minimum z axis deviation", default_value="0.00001"
            ),
            DeclareLaunchArgument(
                "max_z", description="Maximum z axis deviation", default_value="0.6"
            ),
            DeclareLaunchArgument(
                "min_z_score_x",
                description="Minimum z score on cones distribution (x)",
                default_value="-100000000.0",
            ),
            DeclareLaunchArgument(
                "max_z_score_x",
                description="Maximum z score on cones distribution (x)",
                default_value="100000000000.0",
            ),
            DeclareLaunchArgument(
                "min_z_score_y",
                description="Minimum z score on cones distribution (y)",
                default_value="-1000000000.0",
            ),
            DeclareLaunchArgument(
                "max_z_score_y",
                description="Minimum z score on cones distribution (y)",
                default_value="1000000000.0",
            ),
            DeclareLaunchArgument(
                "min_distance_x",
                description="Minimum distance on the x axis for a cluster to be a cone",
                default_value="0.02",  # not on ground tested
            ),
            DeclareLaunchArgument(
                "min_distance_y",
                description="Minimum distance on the y axis for a cluster to be a cone",
                default_value="0.04",  # not on ground tested
            ),
            DeclareLaunchArgument(
                "min_distance_z",
                description="Minimum distance on the z axis for a cluster to be a cone",
                default_value="0.07",  # not on ground tested
            ),
            DeclareLaunchArgument(
                "min_n_points",
                description="Minimum number of points in the cluster for it to be a cone",
                default_value="4",
            ),
            DeclareLaunchArgument(
                "min_confidence",
                description="Minimum number of confidence a cluster needs to be considered a cone",
                default_value="0.90",  # untested
            ),
            DeclareLaunchArgument(
                "height_out_weight",
                description="Weight for how far the cluster is outside the height limit interval (evaluation)",
                default_value="0.15",  # untested
            ),
            DeclareLaunchArgument(
                "height_in_weight",
                description="Weight for how close the cluster is to the limit while inside it (evaluation)",
                default_value="0.10",  # untested
            ),
            DeclareLaunchArgument(
                "cylinder_radius_weight",
                description="Weight for how far the cluster is outside the cylinders radius(XY plane) (evaluation)",
                default_value="0.2",  # untested
            ),
            DeclareLaunchArgument(
                "cylinder_height_weight",
                description="Weight for how far the cluster is outside the cylinders height(Z axis) (evaluation)",
                default_value="0.10",  # untested
            ),
            DeclareLaunchArgument(
                "cylinder_npoints_weight",
                description="Weight for the ratio of points outside the cylinder (evaluation)",
                default_value="0.05",  # untested
            ),
            DeclareLaunchArgument(
                "npoints_weight",
                description="Weight for how low is the number of points in the cluster compared to the minimum (evaluation)",
                default_value="0.2",  # untested
            ),
            DeclareLaunchArgument(
                "displacement_x_weight",
                description="Weight for the displacement in the X direction compared to a minimum (evaluation)",
                default_value="0.03",  # untested
            ),
            DeclareLaunchArgument(
                "displacement_y_weight",
                description="Weight for the displacement in the Y direction compared to a minimum (evaluation)",
                default_value="0.04",  # untested
            ),
            DeclareLaunchArgument(
                "displacement_z_weight",
                description="Weight for the displacement in the Z direction compared to a minimum (evaluation)",
                default_value="0.04",  # untested
            ),
            DeclareLaunchArgument(
                "deviation_xoy_weight",
                description="Weight for how far the cluster is outside the standard deviation in the XOY plane interval (evaluation)",
                default_value="0.07",  # untested
            ),
            DeclareLaunchArgument(
                "deviation_z_weight",
                description="Weight for how far the cluster is outside the standard deviation in the Z axis interval (evaluation)",
                default_value="0.07",  # untested
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception",
                parameters=[
                    {"vehicle_frame_id": LaunchConfiguration("vehicle_frame_id")},
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},
                    {"ransac_iterations": LaunchConfiguration("ransac_iterations")},
                    {"fov_trim_angle": LaunchConfiguration("fov_trim_angle")},
                    {"pc_max_range": LaunchConfiguration("pc_max_range")},
                    {"pc_min_range": LaunchConfiguration("pc_min_range")},
                    {
                        "pc_rlidar_max_height": LaunchConfiguration(
                            "pc_rlidar_max_height"
                        )
                    },
                    {
                        "clustering_n_neighbours": LaunchConfiguration(
                            "clustering_n_neighbours"
                        )
                    },
                    {"clustering_epsilon": LaunchConfiguration("clustering_epsilon")},
                    {
                        "horizontal_resolution": LaunchConfiguration(
                            "horizontal_resolution"
                        )
                    },
                    {"vertical_resolution": LaunchConfiguration("vertical_resolution")},
                    {"adapter": LaunchConfiguration("adapter")},
                    {"ground_removal": LaunchConfiguration("ground_removal")},
                    {"n_angular_grids": LaunchConfiguration("n_angular_grids")},
                    {"radius_resolution": LaunchConfiguration("radius_resolution")},
                    {"target_file": LaunchConfiguration("target_file")},
                    {
                        "max_correspondence_distance": LaunchConfiguration(
                            "max_correspondence_distance"
                        )
                    },
                    {"max_iteration": LaunchConfiguration("max_iteration")},
                    {
                        "transformation_epsilon": LaunchConfiguration(
                            "transformation_epsilon"
                        )
                    },
                    {
                        "euclidean_fitness_epsilon": LaunchConfiguration(
                            "euclidean_fitness_epsilon"
                        )
                    },
                    {
                        "transformation_epsilon": LaunchConfiguration(
                            "transformation_epsilon"
                        )
                    },
                    {"pc_max_range": LaunchConfiguration("pc_max_range")},
                    {"min_height": LaunchConfiguration("min_height")},
                    {"small_max_height": LaunchConfiguration("small_max_height")},
                    {"large_max_height": LaunchConfiguration("large_max_height")},
                    {"min_xoy": LaunchConfiguration("min_xoy")},
                    {"max_xoy": LaunchConfiguration("max_xoy")},
                    {"min_z": LaunchConfiguration("min_z")},
                    {"max_z": LaunchConfiguration("max_z")},
                    {"min_z_score_x": LaunchConfiguration("min_z_score_x")},
                    {"max_z_score_x": LaunchConfiguration("max_z_score_x")},
                    {"min_z_score_y": LaunchConfiguration("min_z_score_y")},
                    {"max_z_score_y": LaunchConfiguration("max_z_score_y")},
                    {"out_distance_cap": LaunchConfiguration("out_distance_cap")},
                    {"min_distance_x": LaunchConfiguration("min_distance_x")},
                    {"min_distance_y": LaunchConfiguration("min_distance_y")},
                    {"min_distance_z": LaunchConfiguration("min_distance_z")},
                    {"min_n_points": LaunchConfiguration("min_n_points")},
                    {"min_confidence": LaunchConfiguration("min_confidence")},
                    {"height_out_weight": LaunchConfiguration("height_out_weight")},
                    {"height_in_weight": LaunchConfiguration("height_in_weight")},
                    {
                        "cylinder_radius_weight": LaunchConfiguration(
                            "cylinder_radius_weight"
                        )
                    },
                    {
                        "cylinder_height_weight": LaunchConfiguration(
                            "cylinder_height_weight"
                        )
                    },
                    {
                        "cylinder_npoints_weight": LaunchConfiguration(
                            "cylinder_npoints_weight"
                        )
                    },
                    {"npoints_weight": LaunchConfiguration("npoints_weight")},
                    {
                        "displacement_x_weight": LaunchConfiguration(
                            "displacement_x_weight"
                        )
                    },
                    {
                        "displacement_y_weight": LaunchConfiguration(
                            "displacement_y_weight"
                        )
                    },
                    {
                        "displacement_z_weight": LaunchConfiguration(
                            "displacement_z_weight"
                        )
                    },
                    {
                        "deviation_xoy_weight": LaunchConfiguration(
                            "deviation_xoy_weight"
                        )
                    },
                    {"deviation_z_weight": LaunchConfiguration("deviation_z_weight")},
                ],
                arguments=["--ros-args", "--log-level", "perception:=debug"],
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception",
                arguments=["--ros-args", "--log-level", "perception:=debug"],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
