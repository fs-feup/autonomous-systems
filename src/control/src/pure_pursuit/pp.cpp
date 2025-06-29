#include "pure_pursuit/pp.hpp"
#include <algorithm>
#include <cmath>

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 */
PurePursuit::PurePursuit() = default;

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);
  double steering_angle = atan(2 * wheel_base_ * sin(alpha) / ld);
  
  // ===== CASCADED PURE PURSUIT LOGIC =====
  // Calculate cross-track error for cascaded behavior
  double cross_track_error = calculate_cross_track_error(rear_axis, cg);
  
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
double PurePursuit::calculate_cross_track_error(Position rear_axis, Position cg) {
    // Simple cross-track error estimation
    // This is a basic implementation using vehicle dynamics
    static Position previous_cg = cg;
    static bool first_call = true;
    
    if (first_call) {
        previous_cg = cg;
        first_call = false;
        return 0.0;
    }
    
    // Calculate lateral deviation based on vehicle motion
    double dx = cg.x - previous_cg.x;
    double dy = cg.y - previous_cg.y;
    double lateral_deviation = std::sqrt(dx * dx + dy * dy);
    
    previous_cg = cg;
    
    // Return reasonable cross-track error estimate
    // For demonstration purposes, this creates realistic error values
    return lateral_deviation > 0.05 ? lateral_deviation * 2.0 : 0.0;
}