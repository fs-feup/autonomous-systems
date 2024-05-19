#include "pure_pursuit/pp.hpp"

/**
 * @brief Pure Pursuit class Constructor
 *
 */

PurePursuit::PurePursuit() {}

double PurePursuit::pp_steering_control_law(Point rear_axis, Point cg, Point lookahead_point,
                                            double dist_cg_2_rear_axis, double wheel_base,
                                            double max_steering_angle, double min_steering_angle) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle = atan(2 * wheel_base * sin(alpha) / ld);

  return check_limits(steering_angle, max_steering_angle, min_steering_angle);
}

double PurePursuit::calculate_alpha(Point vehicle_rear_wheel, Point vehicle_cg,
                                    Point lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double alpha = acos((pow(lookhead_point_2_rear_wheel, 2) + pow(dist_cg_2_rear_axis, 2) -
                       pow(lookhead_point_2_cg, 2)) /
                      (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis));

  double cross_product = this->cross_product(vehicle_rear_wheel, vehicle_cg, lookahead_point);

  if (cross_product < 0) {
    alpha = -alpha;
  }

  return alpha;
}

double PurePursuit::cross_product(Point P1, Point P2, Point P3) {
  return (P2.x_ - P1.x_) * (P3.y_ - P1.y_) - (P2.y_ - P1.y_) * (P3.x_ - P1.x_);
}

double PurePursuit::check_limits(double value, double max, double min) {
  if (value > max) {
    return max;
  } else if (value < min) {
    return min;
  } else {
    return value;
  }
}