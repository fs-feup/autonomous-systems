colcon build --symlink-install

source install/setup.bash

# Perception
ros2 launch yolov5_ros yolov5s_simple.launch.py& --ros-args --log-level warn
ros2 run depth_processing depth_processing_node& --ros-args --log-level warn

# Localisation and Mapping
ros2 run loc_map loc_map& --ros-args --log-level warn

# Planning
ros2 run planning planning& --ros-args --log-level warn

# Control
ros2 run control controller& --ros-args --log-level warn

# Can [eufs, fsds, ads-dv]
ros2 run can can $1& --ros-args --log-level warn
