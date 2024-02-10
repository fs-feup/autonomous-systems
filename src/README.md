# Contributing

This is the src folder of the Autonomous Systems' repository. All the code developed by the team and necessary for execution will be in this folder.


#### Compilation Dependencies

<br>

| Module | Package name | Node name | Compilation command | Running command | 
| ------ | ------------ | --------- | ------------------------ | -------|
| Localization and Mapping | loc_map | loc_map | colcon build --packages-select loc_map custom_interfaces eufs_msgs fs_msgs | ros2 run loc_map loc_map | 
| Path Planning | planning | planning | colcon build --packages-select custom_interfaces planning | ros2 run planning planning |
| Perception | perception | perception | colcon build --packages-select perception custom_interfaces | ros2 run perception perception |
| All | - | - | colcon build --symlink-install | -
| Evaluation Module | plots | plots | colcon build --packages-select plots eufs_msgs custom_interfaces | ros2 run plots plots |
| Mission Control | can | can | colcon build --packages-select can custom_interfaces | ros2 run can can |

<br>
