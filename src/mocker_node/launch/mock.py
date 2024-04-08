from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, DeclareLaunchArgument

def generate_launch_description():
    DeclareLaunchArgument("gtruth_file_path", 
        description="file where the ground truths for planning are stored", 
        default_value="track1.csv"),
    
    return LaunchDescription(
        Node(
            package="mocker_node",
            executable="mocker_node",
            name="mocker_node",
            parameters=[
                {"gtruth_file_path": LaunchConfiguration("gtruth_file_path")},
        ])
    )