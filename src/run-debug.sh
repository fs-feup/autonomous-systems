colcon build --symlink-install

source install/setup.bash

# Perception
ros2 launch yolov5_ros yolov5s_simple.launch.py&
ros2 run depth_processing depth_processing_node&

# Localisation and Mapping
ros2 run loc_map loc_map&

# Planning
ros2 run planning planning&

# Control
ros2 run control controller&

# Can [eufs, fsds, ads-dv]
ros2 run can can $1&
