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
                "ransac_epsilon",
                description="RANSAC epsilon threshold",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "ransac_n_neighbours",
                description="RANSAC number of neighbours",
                default_value="30",
            ),
            DeclareLaunchArgument(
                "clustering_n_neighbours", 
                description="Number of neighbours for Clustering algorithm",
                default_value="3"),
            
            DeclareLaunchArgument(
                "clustering_epsilon", 
                description="Epsilon for Clustering algorithm",
                default_value="0.1"),

            DeclareLaunchArgument(
                "horizontal_resolution", 
                description="Lidar's horizontal resolution",
                default_value="0.2"),

            DeclareLaunchArgument(
                "vertical_resolution", 
                description="Lidar's vertical resolution",
                default_value="0.33"),

            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="vehicle",
            ),

            DeclareLaunchArgument(
                "ground_removal",
                description="Ground Removal algorithm",
                default_value="ransac",
            ),

            DeclareLaunchArgument(
                "n_angular_grids",
                description="Number of angular grids",
                default_value="6",
            ),

            DeclareLaunchArgument(
                "radius_resolution",
                description="radius size of a radius grid (m)",
                default_value="10.0",
            ),

            Node(
                package="perception",
                executable="perception",
                name="perception",
                parameters=[
                    {"ransac_epsilon": LaunchConfiguration("ransac_epsilon")},

                    {"ransac_n_neighbours": LaunchConfiguration("ransac_n_neighbours")},

                    {"clustering_n_neighbours": LaunchConfiguration("clustering_n_neighbours")},

                    {"clustering_epsilon": LaunchConfiguration("clustering_epsilon")},

                    {"horizontal_resolution": LaunchConfiguration("horizontal_resolution")},

                    {"vertical_resolution": LaunchConfiguration("vertical_resolution")},

                    {"adapter": LaunchConfiguration("adapter")},

                    {"ground_removal": LaunchConfiguration("ground_removal")},

                    {"n_angular_grids": LaunchConfiguration("n_angular_grids")},

                    {"radius_resolution": LaunchConfiguration("radius_resolution")}

                ],
                arguments=["--ros-args", "--log-level", "perception:=debug"],
            ),
        ]
    )