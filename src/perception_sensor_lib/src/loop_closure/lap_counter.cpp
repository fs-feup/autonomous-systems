#include "perception_sensor_lib/loop_closure/lap_counter.hpp"
#include <cmath>

LapCounter::LapCounter(double threshold_dist, int first_x_cones)
  : threshold_dist_(threshold_dist),
    first_x_cones_(first_x_cones)
{}

LoopClosure::Result LapCounter::detect(
  const Eigen::Vector3d& current_pose,
  const std::vector<Eigen::Vector2d>& map_cones,
  const Eigen::VectorXi& associations,
  const Eigen::VectorXi& observations) const
{
  double dx = current_pose.x();
  double dy = current_pose.y();
  double dist = std::sqrt(dx * dx + dy * dy);

  // only enable checking after leaving the start region by +1 meter
  if (!searching_) {
    if (dist > threshold_dist_ + 1.0) {
      searching_ = true;
    } else {
      return {false, 0.0};
    }
  }

  // If we're still far from the origin, no closure
  if (dist > threshold_dist_) {
    return {false, 0.0};
  }

  // Look for match with any of the first X cones
  for (int i = 0; i < associations.size(); ++i) {
    int j = associations[i];
    if (j >= 3) {
      int map_idx = (j - 3) / 2;  // Index into map_cones
      if (map_idx < first_x_cones_) {
        return {true, 0.0};
      }
    }
  }

  return {false, 0.0};
}