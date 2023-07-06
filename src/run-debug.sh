colcon build --symlink-install

# Perception
ros2 run perception perception&

# Localisation and Mapping
ros2 run loc_map loc_map&

# Planning
ros2 run planning planning&

# Control
ros2 run control controller&

# Can [eufs, fsds, ads-dv]
ros2 run can can $1&
