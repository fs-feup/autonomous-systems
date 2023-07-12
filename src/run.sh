colcon build --symlink-install

source install/setup.bash

# Perception
ros2 run perception perception& --ros-args --log-level warn

# Localization and Mapping
ros2 run loc_map loc_map& --ros-args --log-level warn

# Planning
ros2 run planning planning& --ros-args --log-level warn

# Control
ros2 run control control& --ros-args --log-level warn

# Can [eufs, fsds, ads-dv]
ros2 run can can $1& --ros-args --log-level warn
