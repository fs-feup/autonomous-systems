from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('factory_robot')
    default_map = os.path.join(pkg_share, 'maps', 'factory2.txt')

    return LaunchDescription([
        Node(
            package='factory_robot',
            executable='factory_node',
            name='factory_node',
            parameters=[{
                'map_path': default_map,
                'publish_rate_hz': 10.0,
            }],
            output='screen'
        )
    ])
