# This message contains information about the state of the EKF
# and is meant for use by the state monitor to ensure the EKF
# isn't failing.

bool gps_received
bool imu_received
bool wheel_odom_received

bool ekf_odom_received
bool ekf_accel_received

bool currently_over_covariance_limit
bool consecutive_turns_over_covariance_limit

bool recommends_failure
