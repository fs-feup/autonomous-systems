import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    get_package_share_directory('yolov5_ros')


    yolov5_ros = launch_ros.actions.Node(
        package="yolov5_ros", executable="yolov5_ros",
        parameters=[
            {"view_img":True},
        ],

    )
    
    return launch.LaunchDescription([
        yolov5_ros,
    ])