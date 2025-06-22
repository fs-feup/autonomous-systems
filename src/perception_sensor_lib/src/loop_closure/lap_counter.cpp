#include "perception_sensor_lib/loop_closure/lap_counter.hpp"

#include <cmath>

LapCounter::LapCounter(double threshold_dist, int first_x_cones, int border_width,
                       int minimum_confidence)
    : threshold_dist_(threshold_dist),
      first_x_cones_(first_x_cones),
      border_width_(border_width),
      minimum_confidence_(minimum_confidence) {}

// TODO: solve this fuck up, not using map_cones and Xi
LoopClosure::Result LapCounter::detect(const Eigen::Vector3d& current_pose,
                                       const Eigen::VectorXd& map_cones,
                                       const Eigen::VectorXi& associations,
                                       const Eigen::VectorXd& observations) const {
  double dx = current_pose.x();
  double dy = current_pose.y();
  double dist = std::sqrt(dx * dx + dy * dy);

  // add a border to give some space until we start searching again
  if (!searching_) {
    if (dist > threshold_dist_ + border_width_) {
      searching_ = true;
    } else {
      return {false, 0.0};
    }
  }

  // If we're still far from the origin, no closure
  if (dist > threshold_dist_) {
    return {false, 0.0};
  }

  int confidence_ = 0;
  // Look for match with any of the first X cones
  for (int i = 0; i < associations.size(); ++i) {
    int j = associations[i];
    if (j >= 0 && j / 2 < first_x_cones_) {  // If observation macthed with one of the first X cones
      confidence_++;  // increase the number of cones that have a match -> increase confidence
    }
  }

  if (confidence_ >= minimum_confidence_) {
    searching_ = false;  // Reset search flag
    return {true, 0.0};
  }

  return {false, 0.0};
}