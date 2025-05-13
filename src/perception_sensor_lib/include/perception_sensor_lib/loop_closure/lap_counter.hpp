#pragma once

#include "loop_closure.hpp"

/**
 * @brief Implementation of LoopClosure that detects when the robot returns near 
 *        the origin and re‑observes any of the first X cones from the map.
 */
class LapCounter : public LoopClosure {
public:
  /**
   * @param threshold_dist   distance (m) around origin to trigger closure
   * @param first_x_cones    consider a loop if you see any of these first X cones
   */
  LapCounter(double threshold_dist, int first_x_cones, int border_width);

  /**
   * @brief Detects loop closure when returning to origin and seeing initial cones
   * @param current_pose     your latest pose in world frame
   * @param map_cones        full map of cones (in insertion order)
   * @param associations     one entry per observation:
   *                         >=3 → matched map_cones[(j-3)/2]
   *                         -1 → new landmark, -2 → no match
   * @param observations     raw observations (unused in this implementation)
   * @return                 {true,0.0} once you re‑see any of map_cones[0..X-1]
   */
  Result detect(
    const Eigen::Vector3d& current_pose,
    const std::vector<Eigen::Vector2d>& map_cones,
    const Eigen::VectorXi& associations,
    const Eigen::VectorXd& observations) const override;

private:
  double threshold_dist_;
  int first_x_cones_;
  int border_width_;
  mutable int confidence_ {0};
  mutable bool searching_{false};
};