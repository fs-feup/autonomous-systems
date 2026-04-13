import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
    SetEnvironmentVariable,
)
from launch.events import Shutdown


def generate_launch_description():
    sdlWaylandEnv = SetEnvironmentVariable(name="SDL_VIDEODRIVER", value="wayland")

    nodeInvictasim = Node(
        package="invictasim",
        namespace="invictasim",
        executable="invictasim",
        name="invictasim_node",
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    nodeInvictasimShutdownEventHandler = RegisterEventHandler(
        OnProcessExit(
            target_action=nodeInvictasim,
            on_exit=[
                LogInfo(msg="Invictasim closed"),
                EmitEvent(
                    event=Shutdown(
                        reason="Invictasim closed, shutdown whole launchfile"
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [sdlWaylandEnv, nodeInvictasim, nodeInvictasimShutdownEventHandler]
    )
