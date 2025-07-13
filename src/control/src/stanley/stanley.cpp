#include "stanley/stanley.hpp"

#include <algorithm>
#include <cmath>

Stanley::Stanley(std::shared_ptr<Filter> lpf, const ControlParameters& params)
    : LateralController(std::move(lpf), params),
      k_(params.stanley_k_),
      epsilon_(params.stanley_epsilon_) {}

double Stanley::steering_control_law(const LateralControlInput& input) {
  // Calculate the path yaw angle based on the closest and next closest point
  double path_yaw = std::atan2(input.next_closest_point.y - input.closest_point.y,
                               input.next_closest_point.x - input.closest_point.x);
  // Calculate the heading error and normalize it
  double heading_error = normalize_angle(path_yaw - input.yaw);

  // Calculate the cross-track error
  double dx = input.cg.x - input.closest_point.x;
  double dy = input.cg.y - input.closest_point.y;
  double cross_track_error = std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;

  // Calculate the cross-track error correction term
  double cte_term = std::atan2(k_ * cross_track_error, input.velocity + epsilon_);

  // Combine the heading error and the cross-track error term to get the steering angle
  double steering = heading_error + cte_term;

  // Apply low-pass filter to the steering angle
  double filtered_steering = lpf_->filter(steering);

  return std::clamp(filtered_steering, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

double Stanley::normalize_angle(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) angle += 2.0 * M_PI;
  return angle - M_PI;
}