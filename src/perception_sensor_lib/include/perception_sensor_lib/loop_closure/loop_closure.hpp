#pragma once

#include <gtsam/geometry/Pose2.h>
#include <Eigen/Core>
#include <common_lib/structures/cone.hpp>
#include <vector>
#include <utility>

/**
 * @brief Detects a loop closure when the robot returns near the origin
 *        and re‑observes any of the first X cones from the map.
 */
class LoopClosure {
public:
  /// (detected?, offset)
  using Result = std::pair<bool, double>;

  /**
   * @param threshold_dist   distance (m) around origin to trigger closure
   * @param first_x_cones    consider a loop if you see any of these first X cones
   */
  LoopClosure(double threshold_dist, int first_x_cones);

  /**
   * @brief Call every time you have new observations.
   * @param current_pose     your latest pose in world frame
   * @param map_cones        full map of cones (in insertion order)
   * @param associations     one entry per observation:
   *                         >=3 → matched map_cones[(j-3)/2]
   *                         -1 → new landmark, -2 → no match
   * @param observations     raw observations (unused here)
   * @return                 {true,0.0} once you re‑see any of map_cones[0..X-1]
   */
  Result detect(
    const gtsam::Pose2& current_pose,
    const std::vector<common_lib::structures::Cone>& map_cones,
    const Eigen::VectorXi& associations,
    const std::vector<common_lib::structures::Cone>& observations) const;

private:
  double threshold_dist_;
  int first_x_cones_;
  mutable bool searching_{false};
};
