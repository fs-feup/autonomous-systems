# Represents a local 5x5 grid view centered on the robot
# Characters: 'R' = robot (center), '0' = floor, '9' = wall, 
#             '1'-'4' = boxes, 'A'-'D' = shelves, '$' = out of bounds

std_msgs/Header header
string[] grid  # 25 characters representing 5x5 grid (row-major order)
int32 size     # Grid size (should be 5)
custom_interfaces/Point2d robot_global_position  # Robot's global position
