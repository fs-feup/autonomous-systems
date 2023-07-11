# This message includes both the kinematic and the dynamic state of the vehicle
# All values are in SI units
std_msgs/Header header
string child_frame_id # aka frame of the car

# Kinematic State (x_k) in header frame
geometry_msgs/PoseWithCovariance pose

# Dynamic State (x_d) in child_frame_id
geometry_msgs/TwistWithCovariance twist   # m/s
geometry_msgs/Vector3 linear_acceleration # m/s^2
float64[9] linear_acceleration_covariance # Row major x, y z in (m/s^2)
float64 slip_angle

# Metadata
float64 state_of_charge # in Volts (V)
