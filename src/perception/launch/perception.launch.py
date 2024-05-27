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
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "ransac_n_neighbours",
                description="RANSAC number of neighbours",
                default_value="10",
            ),
            DeclareLaunchArgument(
                "clustering_n_neighbours",
                description="Number of neighbours for Clustering algorithm",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "clustering_epsilon",
                description="Epsilon for Clustering algorithm",
                default_value="1.0",
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
                default_value="fsds",
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception_node",
                parameters=[
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},
                    {"ransac_n_neighbours": LaunchConfiguration("ransac_n_neighbours")},
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
                ],
                arguments=["--ros-args", "--log-level", "perception:=debug"],
            ),
            Node(
                package="perception",
                executable="perception",
                name="perception",
                parameters=[
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},
                    {"ransac_n_neighbours": LaunchConfiguration("ransac_n_neighbours")},
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
                ],
                arguments=["--ros-args", "--log-level", "perception:=debug"],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
