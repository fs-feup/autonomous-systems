import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown


def generate_launch_description():
    nodeFsfsim = Node(
        package="fsfsim",
        namespace="fsfsim",
        executable="fsfsim",
        name="fsfsim_node",
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    nodeFsfsimShutdownEventHandler = RegisterEventHandler(
        OnProcessExit(
            target_action=nodeFsfsim,
            on_exit=[
                LogInfo(msg="Fsfsim closed"),
                EmitEvent(event=Shutdown(reason="Fsfsim closed, shutdown whole launchfile")),
            ],
        )
    )

    return LaunchDescription([nodeFsfsim, nodeFsfsimShutdownEventHandler])
