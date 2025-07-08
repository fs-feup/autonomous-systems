#include "stanley/stanley.hpp"

#include <algorithm>
#include <cmath>

Stanley::Stanley(std::shared_ptr<Filter> lpf, const ControlParameters& params)
    : LateralController(std::move(lpf), params), stanley_k_(params.stanley_k_) {}

double Stanley::steering_control_law(const LateralControlInput& input) {
  double path_yaw = std::atan2(input.next_closest_point.y - input.closest_point.y,
                               input.next_closest_point.x - input.closest_point.x);

  double heading_error = normalize_angle(path_yaw - input.yaw);

  double dx = input.cg.x - input.closest_point.x;
  double dy = input.cg.y - input.closest_point.y;
  double cross_track_error = std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;

  double velocity = std::max(input.velocity, 0.001);  // avoid divide by zero

  double cte_term = std::atan2(stanley_k_ * cross_track_error, velocity);

  double steering = heading_error + cte_term;

  double filtered_steering = lpf_->filter(steering);
  return std::clamp(filtered_steering, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

// Helper to normalize angle to [-pi, pi]
double Stanley::normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}