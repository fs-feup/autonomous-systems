# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="emergency_remote_control",
                executable="node_remote_control",
                name="remote_control",
                arguments=["--ros-args", "--log-level", "remote_control:=info"],
                output="screen",
                parameters=[
                    {
                        "websocket_host": "0.0.0.0",
                        "websocket_port": 4321,
                        "websocket_idle_timeout_ms": 0,
                        "stale_timeout_ms": 500,
                        "command_period_ms": 25,
                        "velocity_error_min": -2000.0,
                        "velocity_error_max": 2000.0,
                        "ebs_stop_rpm_tolerance": 50.0,
                        "ebs_pid_kp": 0.0002,
                        "ebs_pid_ki": 0.0,
                        "ebs_pid_kd": 0.0,
                        "ebs_pid_output_min": -0.4,
                        "ebs_pid_output_max": 0.4,
                    }
                ],
            ),
        ]
    )
