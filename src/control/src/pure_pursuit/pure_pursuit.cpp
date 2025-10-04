#include "pure_pursuit/pure_pursuit.hpp"

#include <algorithm>

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 *
 */
PurePursuit::PurePursuit(const ControlParameters& params) : params_(std::make_shared<ControlParameters>(params)),
    lpf_(std::make_shared<LowPassFilter>(params.lpf_alpha_, params.lpf_initial_value_)) {}

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle = atan(2 * this->params_->car_parameters_.wheelbase * sin(alpha) / ld);
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle, this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
    this->params_->car_parameters_.steering_parameters.maximum_steering_angle);
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