#include "pure_pursuit/pp.hpp"

#include <algorithm>

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 *
 */

PurePursuit::PurePursuit(std::shared_ptr<Filter> lpf, const ControlParameters& params)
    : LateralController(std::move(lpf), params) {}

double PurePursuit::steering_control_law(const LateralControlInput& input) {
  // Calculate the alpha angle between the vehicle's rear axis and the lookahead point
  double alpha =
      calculate_alpha(input.rear_axis, input.cg, input.lookahead_point, input.dist_cg_2_rear_axis);

  // Calculate the distance from the rear axis to the lookahead point
  double ld = input.rear_axis.euclidean_distance(input.lookahead_point);

  // Calculate the steering angle
  double steering_angle = atan(2 * WHEEL_BASE * sin(alpha) / ld);

  // Apply low-pass filter to the steering angle
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
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