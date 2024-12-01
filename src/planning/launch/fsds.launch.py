# pylint: skip-file
# mypy: ignore-errors
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = "/home/ws/src/planning/config_files/fsds.yaml"

    # Print the parameter file path for debugging
    print(f"Using parameter file: {config_file}")

    # Try to read and print the file contents
    try:
        with open(config_file, "r") as file:
            print("Contents of the parameter file:")
            print(file.read())
    except Exception as e:
        print(f"Failed to read parameter file: {e}")

    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="planning",
                name="planning_adapter",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "planning:=info"],
            ),
        ]
    )
