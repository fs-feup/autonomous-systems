#pragma once

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Data structure to hold motion data
 * @details used to record the velocities received and to redo their processing after optimization
 */
struct MotionData {
  std::shared_ptr<Eigen::Vector3d> velocities_;
  rclcpp::Time timestamp_;

  MotionData() = default;
  MotionData(std::shared_ptr<Eigen::Vector3d> velocities, rclcpp::Time timestamp)
      : velocities_(velocities), timestamp_(timestamp) {}
};

/**
 * @brief Data structure to hold observation data
 * @details used to record the velocities received and to redo their processing after optimization
 */
struct ObservationData {
  std::shared_ptr<Eigen::VectorXd> observations_;
  std::shared_ptr<Eigen::VectorXi> associations_;
  std::shared_ptr<Eigen::VectorXd> observations_global_;
  rclcpp::Time timestamp_;

  ObservationData() = default;
  ObservationData(std::shared_ptr<Eigen::VectorXd> observations,
                  std::shared_ptr<Eigen::VectorXi> associations,
                  std::shared_ptr<Eigen::VectorXd> observations_global, rclcpp::Time timestamp)
      : observations_(observations),
        associations_(associations),
        observations_global_(observations_global),
        timestamp_(timestamp) {}
};