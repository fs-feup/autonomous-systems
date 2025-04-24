#pragma once

#include <gtsam/geometry/Pose2.h>
#include <Eigen/Core>
#include <common_lib/structures/cone.hpp>
#include <vector>
#include <utility>

/**
 * @brief Interface for detecting loop closures
 */
class LoopClosure {
public:

  /**
   * @brief Result of loop closure detection
   * @param detected    true if a loop closure was detected
   * @param offset      offset in meters of the map deviation
   */
  struct Result{
    bool detected;
    double offset;
  };

  virtual ~LoopClosure() = default;

  /**
   * @brief Call every time you have new observations.
   * @param current_pose     your latest pose in world frame
   * @param map_cones        full map of cones (in insertion order)
   * @param associations     one entry per observation:
   *                         >=3 → matched map_cones[(j-3)/2]
   *                         -1 → new landmark, -2 → no match
   * @param observations     raw observations
   * @return                 result indicating if loop closure was detected
   */
  virtual Result detect(
    const gtsam::Pose2& current_pose,
    const std::vector<common_lib::structures::Cone>& map_cones,
    const Eigen::VectorXi& associations,
    const std::vector<common_lib::structures::Cone>& observations) const = 0;
};