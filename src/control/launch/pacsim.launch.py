from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulated_planning",
                description="Wether or not to use Mocker Node for Plannning (true/false)",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Vehicle or Simulation mode (pacsim, fsds, eufs)",
                default_value="pacsim",
            ),
            DeclareLaunchArgument(
                "lookahead_gain",
                description="Variable K -> Lookahead Gain",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "pid_kp",
                description="Proportional Gain of Longitudinal Control",
                default_value="0.4",
            ),
            DeclareLaunchArgument(
                "pid_ki",
                description="Integral Gain of Longitudinal Control",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "pid_kd",
                description="Derivative Gain of Longitudinal Control",
                default_value="0.09",
            ),
            DeclareLaunchArgument(
                "pid_tau",
                description="Derivative low pass filter time constant of Longitudinal Control",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "pid_t",
                description="Sampling period of Longitudinal Control",
                default_value="0.01",
            ),
            DeclareLaunchArgument(
                "pid_lim_min",
                description="Minimum output value of Longitudinal Control",
                default_value="-1",
            ),
            DeclareLaunchArgument(
                "pid_lim_max",
                description="Maximum output value of Longitudinal Control",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "pid_anti_windup",
                description="Anti windup constant of Longitudinal Control",
                default_value="0.7",
            ),
            DeclareLaunchArgument(
                "use_simulated_se",
                description="Use Simulated State Estimation, that is, vehicle state from simulator (true/false)",
                default_value="true",
            ),
            Node(
                package="control",
                executable="node_control",
                name="control_adapter",
                parameters=[
                    {"adapter": LaunchConfiguration("adapter")},
                    {
                        "use_simulated_planning": LaunchConfiguration(
                            "use_simulated_planning"
                        )
                    },
                    {"lookahead_gain": LaunchConfiguration("lookahead_gain")},
                    {"use_simulated_se": LaunchConfiguration("use_simulated_se")},
                ],
                arguments=["--ros-args", "--log-level", "control:=debug"],
            ),
        ]
    )
