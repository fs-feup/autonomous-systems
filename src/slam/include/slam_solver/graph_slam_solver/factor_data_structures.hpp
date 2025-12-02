#pragma once

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Enum to define the type of motion input
 */
enum class MotionInputType {
  VELOCITIES,  ///< Vehicle velocities
  ODOMETRY     ///< Pose difference from odometry
};

/**
 * @brief Data structure to hold motion data
 * @details used to record the velocities received and to redo their processing after optimization
 */
struct MotionData {
  std::shared_ptr<Eigen::VectorXd> motion_data_;
  std::shared_ptr<Eigen::VectorXd> motion_data_noise_;
  rclcpp::Time timestamp_;
  MotionInputType type_ = MotionInputType::VELOCITIES;  ///< Type of motion input

  MotionData() = default;
  MotionData(std::shared_ptr<Eigen::VectorXd> motion_data,
             std::shared_ptr<Eigen::VectorXd> motion_data_noise, rclcpp::Time timestamp,
             MotionInputType type = MotionInputType::VELOCITIES)
      : motion_data_(motion_data),
        motion_data_noise_(motion_data_noise),
        timestamp_(timestamp),
        type_(type) {}
};

/**
 * @brief Data structure to hold observation data
 * @details used to record the cone observations received and to redo their processing after
 * optimization
 */
struct ObservationData {
  std::shared_ptr<Eigen::VectorXd> observations_;
  std::shared_ptr<Eigen::VectorXi> associations_;
  std::shared_ptr<Eigen::VectorXd> observations_global_;
  std::shared_ptr<Eigen::VectorXd> observations_confidences_;
  rclcpp::Time timestamp_;

  ObservationData() = default;
  ObservationData(std::shared_ptr<Eigen::VectorXd> observations,
                  std::shared_ptr<Eigen::VectorXi> associations,
                  std::shared_ptr<Eigen::VectorXd> observations_global,
                  std::shared_ptr<Eigen::VectorXd> observations_confidences, rclcpp::Time timestamp)
      : observations_(observations),
        associations_(associations),
        observations_global_(observations_global),
        observations_confidences_(observations_confidences),
        timestamp_(timestamp) {}
};