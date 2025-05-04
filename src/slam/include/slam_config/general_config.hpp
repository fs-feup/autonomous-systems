#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "perception_sensor_lib/data_association/parameters.hpp"

/**
 * @brief Parameters for the SLAM node
 */
struct SLAMParameters {
  bool use_simulated_perception_ = false;
  bool use_simulated_velocities_ = false;
  std::string motion_model_name_ = "constant_velocity";
  std::string data_association_model_name_ = "nearest_neighbor";
  std::string slam_solver_name_ = "graph_slam";
  std::string frame_id_ = "map";
  float data_association_limit_distance_ = 70;
  float observation_x_noise_ = 0.01;
  float observation_y_noise_ = 0.01;
  float velocity_x_noise_ = 0.1;
  float velocity_y_noise_ = 0.1;
  float angular_velocity_noise_ = 0.1;
  double data_association_gate_ = 1.23;
  double new_landmark_confidence_gate_ = 0.6;
  double slam_min_pose_difference_ =
      0.3;  //< Minimum pose difference to add a new pose to the graph
  double slam_optimization_period_ = 0.0;  //< Period for running optimization of the graph (s),
                                           //  0.0 means optimization on observations receival
  std::string slam_optimization_type_ = "normal_levenberg";
  std::string slam_optimization_mode_ = "sync";

  double slam_isam2_relinearize_threshold_ = 0.1;
  double slam_isam2_relinearize_skip_ = 1;
  std::string slam_isam2_factorization_ = "QR";
  unsigned int sliding_window_size_ = 5;

  SLAMParameters() = default;
  SLAMParameters(const SLAMParameters &params);

  /**
   * @brief Load the configuration for the SLAM node from YAML file
   *
   * @return std::string adapter_name
   */
  std::string load_config();

  DataAssociationParameters get_data_association_parameters() {
    return DataAssociationParameters(data_association_limit_distance_, data_association_gate_,
                                     new_landmark_confidence_gate_, observation_x_noise_,
                                     observation_y_noise_);
  }

  friend std::ostream &operator<<(std::ostream &os, const SLAMParameters &params) {
    os << "SLAMParameters: {"
       << ", frame_id_: " << params.frame_id_
       << "use_simulated_perception_: " << params.use_simulated_perception_
       << ", use_simulated_velocities_: " << params.use_simulated_velocities_
       << ", motion_model_name_: " << params.motion_model_name_
       << ", data_association_model_name_: " << params.data_association_model_name_
       << ", data_association_limit_distance_: " << params.data_association_limit_distance_
       << ", data_association_gate_: " << params.data_association_gate_
       << ", new_landmark_confidence_gate_: " << params.new_landmark_confidence_gate_
       << ", observation_x_noise_: " << params.observation_x_noise_
       << ", observation_y_noise_: " << params.observation_y_noise_
       << ", velocity_x_noise_: " << params.velocity_x_noise_
       << ", velocity_y_noise_: " << params.velocity_y_noise_
       << ", angular_velocity_noise_: " << params.angular_velocity_noise_
       << ", slam_solver_name_: " << params.slam_solver_name_
       << ", slam_min_pose_difference_: " << params.slam_min_pose_difference_
       << ", slam_optimization_mode_: " << params.slam_optimization_mode_
       << ", slam_optimization_type_: " << params.slam_optimization_type_
       << ", slam_optimization_period_: " << params.slam_optimization_period_
       << ", sliding_window_size_: " << params.sliding_window_size_
       << ", slam_isam2_relinearize_threshold_: " << params.slam_isam2_relinearize_threshold_
       << ", slam_isam2_relinearize_skip_: " << params.slam_isam2_relinearize_skip_
       << ", slam_isam2_factorization_: " << params.slam_isam2_factorization_ << "}";
    return os;
  }
};