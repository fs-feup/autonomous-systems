# Contains all objects perceived within the robot's perception radius

std_msgs/Header header
custom_interfaces/PerceivedObject[] objects  # List of all perceived objects
float64 perception_radius  # The radius within which objects are detected
custom_interfaces/Point2d robot_position  # Robot's global position for reference
