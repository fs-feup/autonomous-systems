from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package='perception',
            executable='perception',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'perception:=debug']
        ),
        Node(
            package='loc_map',
            executable='loc_map',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'loc_map:=debug']
        ),
        Node(
            package='planning',
            executable='planning',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'planning:=debug']
        ),
        Node(
            package='control',
            executable='control',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'control:=debug']
        ),
        # Node(
        #     package="ros_can",
        #     executable="ros_can_node",
        #     name="ros_can",
        #     parameters=[
        #         {"use_sim_time": False},
        #         {"can_debug": 0},
        #         {"simulate_can": 0},
        #         {"can_interface": "can0"},
        #         {"loop_rate": 100},
        #         {"rpm_limit": 1000},
        #         {"max_acc": 5.0},
        #         {"max_braking": 5.0},
        #         {"cmd_timeout": 0.5}
        # ])
    ]

    return LaunchDescription(nodes)