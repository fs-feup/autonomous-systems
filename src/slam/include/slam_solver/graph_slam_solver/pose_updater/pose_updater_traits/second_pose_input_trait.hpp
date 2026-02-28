#pragma once

#include <Eigen/Dense>

/**
 * @brief Trait for pose updaters that keep two different pose difference estimates from different
 * sources
 * @details This trait defines the interface for pose updaters that keep two different pose
 * difference estimates from different sources e.g. keeping one from odometry and one from velocity
 * estimation and fusing both through factors in the graph
 */
class SecondPoseInputTrait {
  int _last_pose_number_ = 0;

public:
  /**
   * @brief Get the accumulated pose difference from the second source
   * @return Eigen::Vector3d The accumulated pose difference from the second source
   */
  virtual Eigen::Vector3d get_second_accumulated_pose_difference() const = 0;

  /**
   * @brief Check if the second pose difference is ready
   * @return true if the second pose difference is ready, false otherwise
   */
  virtual bool second_pose_difference_ready() const = 0;
};