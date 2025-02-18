# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "estimation_method",
                description="Estimation method to use",
                default_value="ekf",
            ),
            DeclareLaunchArgument(
                "adapter",
                description="Environment to run node on",
                default_value="pacsim",
            ),
            DeclareLaunchArgument(
                "ekf_process_noise",
                description="Process noise for EKF",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "ekf_measurement_noise",
                description="Process noise for EKF",
                default_value="0.7",
            ),
            DeclareLaunchArgument(
                "wheel_base",
                description="distance between front and rear wheels",
                default_value="1.6",
            ),
            DeclareLaunchArgument(
                "weight_distribution_front",
                description="weight distribution on front wheels",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "gear_ratio",
                description="Motor to wheel gear ratio",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "wheel_radius",
                description="Wheel radius",
                default_value="0.258",
            ),            
            Node(
                package="velocity_estimation",
                executable="velocity_estimation",
                name="temporary_ve_adapter",
                output="screen",
                parameters=[
                    {"estimation_method": LaunchConfiguration("estimation_method")},
                    {"adapter": LaunchConfiguration("adapter")},
<<<<<<< HEAD
                    {"ekf_process_noise": LaunchConfiguration("ekf_process_noise")},                    
=======
                    {"ekf_process_noise": LaunchConfiguration("ekf_process_noise")},  
                    {"ekf_measurement_noise": LaunchConfiguration("ekf_measurement_noise")},
                    {"wheel_base": LaunchConfiguration("wheel_base")},  
                    {"weight_distribution_front": LaunchConfiguration("weight_distribution_front")},  
                    {"gear_ratio": LaunchConfiguration("gear_ratio")},  
                    {"wheel_radius": LaunchConfiguration("wheel_radius")},                  
>>>>>>> dev
                ],
                arguments=["--ros-args", "--log-level", "velocity_estimation:=info"],
            ),
        ]
    )
