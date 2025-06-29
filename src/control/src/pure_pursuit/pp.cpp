#include "pure_pursuit/pp.hpp"

#include <algorithm>
#include <cmath>

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 */
PurePursuit::PurePursuit(std::shared_ptr<Filter> lpf) : lpf_(std::move(lpf)) {}

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);

  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);
  double steering_angle = atan(2 * wheel_base_ * sin(alpha) / ld);

  // ===== CASCADED PURE PURSUIT LOGIC =====
  // Calculate cross-track error for cascaded behavior
  double cross_track_error = calculate_cross_track_error(rear_axis, cg, lookahead_point);

  // Determine steering limits based on cross-track error
  double max_steering, min_steering;

  if (std::abs(cross_track_error) > emergency_cross_track_threshold_) {
    // CASCADED MODE: Emergency steering limits
    max_steering = emergency_max_steering_angle_;
    min_steering = -emergency_max_steering_angle_;
    cascaded_mode_active_ = true;

    // Optional logging for verification
    static int log_counter = 0;
    if (log_counter++ % 25 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("pure_pursuit"),
                  "CASCADED MODE: Cross-track error %.2fm > %.2fm, emergency steering %.2f rad",
                  cross_track_error, emergency_cross_track_threshold_, max_steering);
    }
  } else {
    // NORMAL MODE: Standard steering limits
    max_steering = max_steering_angle_;
    min_steering = min_steering_angle_;
    cascaded_mode_active_ = false;
  }

  return std::clamp(steering_angle, min_steering, max_steering);
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle, min_steering_angle_, max_steering_angle_);
}

double PurePursuit::calculate_alpha(Position vehicle_rear_wheel, Position vehicle_cg,
                                    Position lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double cos_alpha = (pow(lookhead_point_2_rear_wheel, 2) + pow(dist_cg_2_rear_axis, 2) -
                      pow(lookhead_point_2_cg, 2)) /
                     (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis);

  cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
  double alpha = acos(cos_alpha);

  if (cross_product(vehicle_rear_wheel, vehicle_cg, lookahead_point) < 0) {
    alpha = -alpha;
  }

  return alpha;
}

// ===== CASCADED PURE PURSUIT HELPER FUNCTION =====
double PurePursuit::calculate_cross_track_error(Position rear_axis, Position cg, Position lookahead_point) {
     // REAL cross-track error: distance from vehicle to desired path
    
    // Path vector: from rear_axis to lookahead_point (desired direction)
    double path_dx = lookahead_point.x - rear_axis.x;
    double path_dy = lookahead_point.y - rear_axis.y;
    double path_length = sqrt(path_dx * path_dx + path_dy * path_dy);
    
    // If too close to target, no meaningful cross-track error
    if (path_length < 0.5) {
        return 0.0;
    }
    
    // Unit vector along desired path direction
    double path_unit_x = path_dx / path_length;
    double path_unit_y = path_dy / path_length;
    
    // Current vehicle position relative to path start (rear_axis)
    double vehicle_dx = cg.x - rear_axis.x;
    double vehicle_dy = cg.y - rear_axis.y;
    
    // Cross product gives perpendicular distance to path line
    // This is the REAL cross-track error (lateral deviation from desired path)
    double cross_track_error = std::abs(vehicle_dx * (-path_unit_y) + vehicle_dy * path_unit_x);
    
    return cross_track_error;  // Returns distance in meters
}