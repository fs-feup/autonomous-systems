#include "loop_closure.hpp"
#include <cmath>

LoopClosure::LoopClosure(double threshold_dist, int first_x_cones)
  : threshold_dist_(threshold_dist),
    first_x_cones_(first_x_cones)
{}

LoopClosure::Result LoopClosure::detect(
  const gtsam::Pose2&                          current_pose,
  const std::vector<common_lib::structures::Cone>& map_cones,
  const Eigen::VectorXi&                       associations,
  const std::vector<common_lib::structures::Cone>& observations) const
{
  if (loop_closure_detected_) {
    return {true, 0.0};
  }

  double dx = current_pose.x();
  double dy = current_pose.y();
  double dist = std::sqrt(dx*dx + dy*dy);

  // only start checking after leaving the start region by +1 meter
  if (!searching_) {
    if (dist > threshold_dist_ + 1.0) {
      searching_ = true;
    } else {
      return {false, 0.0};
    }
  }

  if (dist > threshold_dist_) {
    return {false, 0.0};
  }

  // for each observation, see if it matched one of the first X cones
  for (int i = 0; i < associations.size(); ++i) {
    int j = associations[i];
    if (j >= 3) {
      int map_idx = (j - 3) / 2;       // 0-based index into map_cones
      if (map_idx < first_x_cones_) {
        loop_closure_detected_ = true;
        return {true, 0.0};
      }
    }
  }

  return {false, 0.0};
}
