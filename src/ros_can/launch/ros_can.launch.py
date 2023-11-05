from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
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
            {"max_acc": 50.0},
            {"max_braking": 5.0},
            {"cmd_timeout": 100000000000000.0}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
