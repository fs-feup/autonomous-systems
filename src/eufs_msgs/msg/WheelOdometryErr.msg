# Message for the error of Wheel Odometry. Error based on euclidean distance.

std_msgs/Header header

float64 position_err
float64 orientation_err
float64 linear_vel_err
float64 angular_vel_err
float64 forward_vel_err # Just a-axis motion, since that has special importance for Wheel Odometry
