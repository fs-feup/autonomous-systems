#include <algorithm>
#include "pure_pursuit/pp.hpp"

/**
 * @brief Pure Pursuit class Constructor
 *
 */

PurePursuit::PurePursuit() = default;

double PurePursuit::pp_steering_control_law(Point rear_axis, Point cg, Point lookahead_point,
                                            double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle = atan(2 * wheel_base_ * sin(alpha) / ld);
  return std::clamp(steering_angle, min_steering_angle_, max_steering_angle_);
}

double PurePursuit::calculate_alpha(Point vehicle_rear_wheel, Point vehicle_cg,
                                    Point lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double alpha = acos((pow(lookhead_point_2_rear_wheel, 2) + pow(dist_cg_2_rear_axis, 2) -
                       pow(lookhead_point_2_cg, 2)) /
                      (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis));

  if (double cross_product = this->cross_product(vehicle_rear_wheel, vehicle_cg, lookahead_point);
      cross_product < 0) {
    alpha = -alpha;
  }

  return alpha;
}

double PurePursuit::cross_product(Point p1, Point p2, Point p3) const {
  return (p2.x_ - p1.x_) * (p3.y_ - p1.y_) - (p2.y_ - p1.y_) * (p3.x_ - p1.x_);
}