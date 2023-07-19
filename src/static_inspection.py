from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package='control',
            executable='control',
            output='screen'
        ),
        Node(
            package="ros_can",
            executable="ros_can_node",
            name="ros_can",
            parameters=[
                {"use_sim_time": False},
                {"can_debug": 0},
                {"simulate_can": 0},
                {"can_interface": "can0"},
                {"loop_rate": 100},
                {"rpm_limit": 1000},
                {"max_acc": 15.0},
                {"max_braking": 5.0},
                {"cmd_timeout": 0.5}
        ])
    ]

    return LaunchDescription(nodes)