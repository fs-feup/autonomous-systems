#include "perception_sensor_lib/loop_closure/lap_counter.hpp"
#include <cmath>

LapCounter::LapCounter(double threshold_dist, int first_x_cones, int border_width)
  : threshold_dist_(threshold_dist),
    first_x_cones_(first_x_cones),
    border_width_(border_width)
{}

LoopClosure::Result LapCounter::detect(
  const Eigen::Vector3d& current_pose,
  const std::vector<Eigen::Vector2d>& map_cones,
  const Eigen::VectorXi& associations,
  const Eigen::VectorXd& observations) const
{
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

  bool match_found = false;
  // Look for match with any of the first X cones
  for (int i = 0; i < associations.size(); ++i) {
    int j = associations[i];
    if (j >= 3) {
      int map_idx = (j - 3) / 2;  // Index into map_cones
      if (map_idx < first_x_cones_) {
        match_found = true;  // Update the outer match_found variable
        break;  // We found a match, no need to continue searching
      }
    }
  }
  
  if (match_found) {
    confidence_++;
    if (confidence_ >= 3) {
      // We found loop closure, reset confidence and return
      confidence_ = 0;
      searching_ = false;  // Reset search flag
      return {true, 0.0};
    }
  } else {
    confidence_ = 0;
  }
  
  return {false, 0.0};
}