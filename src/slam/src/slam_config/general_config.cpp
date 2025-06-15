#include "slam_config/general_config.hpp"

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

SLAMParameters::SLAMParameters(const SLAMParameters &params) {
  use_simulated_perception_ = params.use_simulated_perception_;
  use_simulated_velocities_ = params.use_simulated_velocities_;
  motion_model_name_ = params.motion_model_name_;
  data_association_model_name_ = params.data_association_model_name_;
  data_association_limit_distance_ = params.data_association_limit_distance_;
  observation_x_noise_ = params.observation_x_noise_;
  observation_y_noise_ = params.observation_y_noise_;
  velocity_x_noise_ = params.velocity_x_noise_;
  velocity_y_noise_ = params.velocity_y_noise_;
  angular_velocity_noise_ = params.angular_velocity_noise_;
  pose_x_initial_noise_ = params.pose_x_initial_noise_;
  pose_y_initial_noise_ = params.pose_y_initial_noise_;
  pose_theta_initial_noise_ = params.pose_theta_initial_noise_;
  slam_solver_name_ = params.slam_solver_name_;
  slam_min_pose_difference_ = params.slam_min_pose_difference_;
  slam_optimization_period_ = params.slam_optimization_period_;
  landmark_filter_name_ = params.landmark_filter_name_;
  minimum_frequency_of_detections_ = params.minimum_frequency_of_detections_;
  minimum_observation_count_ = params.minimum_observation_count_;
  frame_id_ = params.frame_id_;
  slam_optimization_type_ = params.slam_optimization_type_;
  slam_optimization_mode_ = params.slam_optimization_mode_;
  preloaded_map_noise_ = params.preloaded_map_noise_;
  slam_isam2_relinearize_threshold_ = params.slam_isam2_relinearize_threshold_;
  slam_isam2_relinearize_skip_ = params.slam_isam2_relinearize_skip_;
  slam_isam2_factorization_ = params.slam_isam2_factorization_;
  sliding_window_size_ = params.sliding_window_size_;
}

SLAMParameters &SLAMParameters::operator=(const SLAMParameters &other) {
  if (this != &other) {
    use_simulated_perception_ = other.use_simulated_perception_;
    use_simulated_velocities_ = other.use_simulated_velocities_;
    motion_model_name_ = other.motion_model_name_;
    data_association_model_name_ = other.data_association_model_name_;
    data_association_limit_distance_ = other.data_association_limit_distance_;
    observation_x_noise_ = other.observation_x_noise_;
    observation_y_noise_ = other.observation_y_noise_;
    velocity_x_noise_ = other.velocity_x_noise_;
    velocity_y_noise_ = other.velocity_y_noise_;
    angular_velocity_noise_ = other.angular_velocity_noise_;
    pose_x_initial_noise_ = other.pose_x_initial_noise_;
    pose_y_initial_noise_ = other.pose_y_initial_noise_;
    pose_theta_initial_noise_ = other.pose_theta_initial_noise_;
    slam_solver_name_ = other.slam_solver_name_;
    slam_min_pose_difference_ = other.slam_min_pose_difference_;
    slam_optimization_period_ = other.slam_optimization_period_;
    frame_id_ = other.frame_id_;
    slam_optimization_type_ = other.slam_optimization_type_;
    slam_optimization_mode_ = other.slam_optimization_mode_;
    preloaded_map_noise_ = other.preloaded_map_noise_;
    slam_isam2_relinearize_threshold_ = other.slam_isam2_relinearize_threshold_;
    slam_isam2_relinearize_skip_ = other.slam_isam2_relinearize_skip_;
    slam_isam2_factorization_ = other.slam_isam2_factorization_;
    sliding_window_size_ = other.sliding_window_size_;
  }
  return *this;
}

std::string SLAMParameters::load_config() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("slam", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string adapter = global_config["global"]["adapter"].as<std::string>();
  this->use_simulated_velocities_ = global_config["global"]["use_simulated_velocities"].as<bool>();
  this->use_simulated_perception_ = global_config["global"]["use_simulated_perception"].as<bool>();
  this->frame_id_ = global_config["global"]["vehicle_frame_id"].as<std::string>();

  std::string slam_config_path =
      common_lib::config_load::get_config_yaml_path("slam", "slam", adapter);

  YAML::Node slam_config = YAML::LoadFile(slam_config_path);

  this->motion_model_name_ = slam_config["slam"]["motion_model_name"].as<std::string>();
  this->data_association_model_name_ =
      slam_config["slam"]["data_association_model_name"].as<std::string>();
  this->data_association_limit_distance_ =
      slam_config["slam"]["data_association_limit_distance"].as<float>();
  this->data_association_gate_ = slam_config["slam"]["data_association_gate"].as<double>();
  this->new_landmark_confidence_gate_ =
      slam_config["slam"]["new_landmark_confidence_gate"].as<double>();
  this->observation_x_noise_ = slam_config["slam"]["observation_x_noise"].as<float>();
  this->observation_y_noise_ = slam_config["slam"]["observation_y_noise"].as<float>();
  this->velocity_x_noise_ = slam_config["slam"]["velocity_x_noise"].as<float>();
  this->velocity_y_noise_ = slam_config["slam"]["velocity_y_noise"].as<float>();
  this->angular_velocity_noise_ = slam_config["slam"]["angular_velocity_noise"].as<float>();
  this->pose_x_initial_noise_ = slam_config["slam"]["pose_x_initial_noise"].as<double>();
  this->pose_y_initial_noise_ = slam_config["slam"]["pose_y_initial_noise"].as<double>();
  this->pose_theta_initial_noise_ = slam_config["slam"]["pose_theta_initial_noise"].as<double>();
  this->slam_solver_name_ = slam_config["slam"]["slam_solver_name"].as<std::string>();
  this->slam_min_pose_difference_ = slam_config["slam"]["slam_min_pose_difference"].as<float>();
  this->slam_optimization_period_ = slam_config["slam"]["slam_optimization_period"].as<double>();
  this->landmark_filter_name_ = slam_config["slam"]["landmark_filter_name"].as<std::string>();
  this->minimum_observation_count_ = slam_config["slam"]["minimum_observation_count"].as<int>();
  this->minimum_frequency_of_detections_ =
      slam_config["slam"]["minimum_frequency_of_detections"].as<double>();
  this->slam_optimization_type_ = slam_config["slam"]["slam_optimization_type"].as<std::string>();
  this->slam_optimization_mode_ = slam_config["slam"]["slam_optimization_mode"].as<std::string>();
  this->preloaded_map_noise_ = slam_config["slam"]["preloaded_map_noise"].as<double>();
  this->slam_isam2_relinearize_threshold_ =
      slam_config["slam"]["slam_isam2_relinearize_threshold"].as<double>();
  this->slam_isam2_relinearize_skip_ =
      slam_config["slam"]["slam_isam2_relinearize_skip"].as<double>();
  this->slam_isam2_factorization_ =
      slam_config["slam"]["slam_isam2_factorization"].as<std::string>();
  this->sliding_window_size_ = slam_config["slam"]["sliding_window_size"].as<unsigned int>();
  return adapter;
}