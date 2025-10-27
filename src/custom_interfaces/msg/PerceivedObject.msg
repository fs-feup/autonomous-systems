# Represents a single perceived object in the robot's local reference frame

string object_type  # "box", "shelf", "wall", "floor"
string object_id    # "1", "2", "3", "4" for boxes; "A", "B", "C", "D" for shelves; "9" for walls; "0" for floor
custom_interfaces/Point2d relative_position  # Position relative to robot (x, y)
