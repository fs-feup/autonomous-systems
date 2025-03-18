#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @brief Parameters for the SLAM node
 */
struct SLAMParameters {
  bool use_simulated_perception_ = false;
  bool use_simulated_velocities_ = false;
  std::string motion_model_name_;
  std::string data_association_model_name_;
  std::string slam_solver_name_;
  float data_association_limit_distance_;
  float observation_x_noise_ = 0.01;
  float observation_y_noise_ = 0.01;
  float velocity_x_noise_ = 0.1;
  float velocity_y_noise_ = 0.1;
  float angular_velocity_noise_ = 0.1;
  double data_association_gate_;
  double new_landmark_confidence_gate_;
  double slam_min_pose_difference_;  //< Minimum pose difference to add a new pose to the graph
  double slam_optimization_period_ = 0.0;  //< Period for running optimization of the graph (s),
                                           //  0.0 means optimization on observations receival

  SLAMParameters() = default;
  explicit SLAMParameters(const SLAMParameters &params);

  /**
   * @brief Load the configuration for the SLAM node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();
};