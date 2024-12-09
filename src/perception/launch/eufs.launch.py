from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "horizontal_resolution",
                description="Lidar's horizontal resolution",
                default_value="0.2",
            ),
            DeclareLaunchArgument(
                "vertical_resolution",
                description="Lidar's vertical resolution",
                default_value="0.33",
            ),
            DeclareLaunchArgument(
                "fov_trim_angle",
                description="Trim the points received to a max angle",
                default_value="60",  # degrees
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
                default_value="-0.20",
            ),
            DeclareLaunchArgument(
                "ground_removal",
                description="Ground Removal algorithm",
                default_value="ransac",
            ),
            DeclareLaunchArgument(
                "n_angular_grids",
                description="Number of angular grids",
                default_value="12",
            ),
            DeclareLaunchArgument(
                "radius_resolution",
                description="Radius size of a radius grid (m)",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "ransac_epsilon",
                description="RANSAC epsilon threshold",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "ransac_n_neighbours",
                description="RANSAC number of neighbours",
                default_value="15",
            ),
            DeclareLaunchArgument(
                "clustering_n_neighbours",
                description="Number of neighbours for Clustering algorithm",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "clustering_epsilon",
                description="Epsilon for Clustering algorithm",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
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
                default_value="-100.0",
            ),
            DeclareLaunchArgument(
                "large_max_height",
                description="Maximum height of a cluster to be considered a large cone",
                default_value="150",  # untested
            ),
            DeclareLaunchArgument(
                "small_max_height",
                description="Maximum height of a cluster to be considered a small cone",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "min_xoy",
                description="Minimum xOy plane deviation",
                default_value="-100.0",
            ),
            DeclareLaunchArgument(
                "max_xoy",
                description="Maximum xOy plane deviation",
                default_value="100.0",
            ),
            DeclareLaunchArgument(
                "min_z", description="Minimum z axis deviation", default_value="-100.0"
            ),
            DeclareLaunchArgument(
                "max_z", description="Maximum z axis deviation", default_value="100.0"
            ),
            DeclareLaunchArgument(
                "min_z_score_x",
                description="Minimum z score on cones distribution (x)",
                default_value="-100.0",
            ),
            DeclareLaunchArgument(
                "max_z_score_x",
                description="Maximum z score on cones distribution (x)",
                default_value="100.0",
            ),
            DeclareLaunchArgument(
                "min_z_score_y",
                description="Minimum z score on cones distribution (y)",
                default_value="-100.0",
            ),
            DeclareLaunchArgument(
                "max_z_score_y",
                description="Minimum z score on cones distribution (y)",
                default_value="100.0",
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception_adapter",
                parameters=[
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},
                    {"ransac_n_neighbours": LaunchConfiguration("ransac_n_neighbours")},
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
                    {"max_height": LaunchConfiguration("max_height")},
                    {"min_xoy": LaunchConfiguration("min_xoy")},
                    {"max_xoy": LaunchConfiguration("max_xoy")},
                    {"min_z": LaunchConfiguration("min_z")},
                    {"max_z": LaunchConfiguration("max_z")},
                    {"min_z_score_x": LaunchConfiguration("min_z_score_x")},
                    {"max_z_score_x": LaunchConfiguration("max_z_score_x")},
                    {"min_z_score_y": LaunchConfiguration("min_z_score_y")},
                    {"max_z_score_y": LaunchConfiguration("max_z_score_y")},
                ],
                arguments=["--ros-args", "--log-level", "perception:=info"],
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception",
                arguments=["--ros-args", "--log-level", "perception:=info"],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
