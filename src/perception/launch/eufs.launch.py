from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ransac_epsilon",
                description="RANSAC epsilon threshold",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "ransac_n_neighbours",
                description="RANSAC number of neighbours",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "fov_trim",
                description="Trim the points received to a max angle",
                default_value="90",  # degrees
            ),
            DeclareLaunchArgument(
                "pc_max_range",
                description="Point cloud filtering based on distance (m)",
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
                default_value="0.45",
            ),
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
                "adapter",
                description="Environment to run node on",
                default_value="eufs",
            ),
            DeclareLaunchArgument(
                "ground_removal",
                description="Ground Removal algorithm",
                default_value="grid_ransac",
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
            Node(
                package="perception",
                executable="perception",
                name="perception_adapter",
                parameters=[
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},
                    {"ransac_n_neighbours": LaunchConfiguration("ransac_n_neighbours")},
                    {"fov_trim": LaunchConfiguration("fov_trim")},
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
                        "transformation_epsilon" : LaunchConfiguration(
                            "transformation_epsilon"
                        )
                    },
                    {
                        "pc_max_range" : LaunchConfiguration(
                            "pc_max_range"
                        )
                    }
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
