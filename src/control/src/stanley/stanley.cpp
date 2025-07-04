#include "stanley/stanley.hpp"

#include <algorithm>
#include <cmath>

Stanley::Stanley(std::shared_ptr<Filter> lpf, const ControlParameters& params)
    : LateralController(std::move(lpf), params), stanley_k_(params.stanley_k_) {}

double Stanley::steering_control_law(const LateralControlInput& input) {
  double path_yaw = std::atan2(input.next_closest_point.y - input.closest_point.y,
                               input.next_closest_point.x - input.closest_point.x);

  // Heading error (difference between vehicle yaw and path yaw at closest point)
  double heading_error = normalize_angle(input.yaw - path_yaw);

  // Cross-track error (distance from CG to closest point, sign based on path orientation)
  double dx = input.closest_point.x - input.cg.x;
  double dy = input.closest_point.y - input.cg.y;
  double cross_track_error = dx * std::sin(path_yaw) - dy * std::cos(path_yaw);

  // Stanley control law
  double velocity = std::max(input.velocity, 0.001);  // avoid divide by zero
  double cte_term = std::atan2(stanley_k_ * cross_track_error, velocity);

  double steering = heading_error + cte_term;

  // Filter and clamp
  double filtered_steering = lpf_ ? lpf_->filter(steering) : steering;
  return std::clamp(filtered_steering, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

// Helper to normalize angle to [-pi, pi]
double Stanley::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}