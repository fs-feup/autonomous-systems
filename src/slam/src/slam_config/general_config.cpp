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
  slam_solver_name_ = params.slam_solver_name_;
}

std::string SLAMParameters::load_config() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("slam", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string adapter = global_config["global"]["adapter"].as<std::string>();
  this->use_simulated_velocities_ = global_config["global"]["use_simulated_velocities"].as<bool>();
  this->use_simulated_perception_ = global_config["global"]["use_simulated_perception"].as<bool>();

  std::string slam_config_path =
      common_lib::config_load::get_config_yaml_path("slam", "slam", adapter);

  YAML::Node slam_config = YAML::LoadFile(slam_config_path);

  this->motion_model_name_ = slam_config["slam"]["motion_model_name"].as<std::string>();
  this->data_association_model_name_ =
      slam_config["slam"]["data_association_model_name"].as<std::string>();
  this->data_association_limit_distance_ =
      slam_config["slam"]["data_association_limit_distance"].as<float>();
  this->data_association_gate_ = slam_config["slam"]["data_association_gate"].as<float>();
  this->new_landmark_confidence_gate_ =
      slam_config["slam"]["new_landmark_confidence_gate"].as<float>();
  this->observation_x_noise_ = slam_config["slam"]["observation_x_noise"].as<float>();
  this->observation_y_noise_ = slam_config["slam"]["observation_y_noise"].as<float>();
  this->velocity_x_noise_ = slam_config["slam"]["velocity_x_noise"].as<float>();
  this->velocity_y_noise_ = slam_config["slam"]["velocity_y_noise"].as<float>();
  this->angular_velocity_noise_ = slam_config["slam"]["angular_velocity_noise"].as<float>();
  this->slam_solver_name_ = slam_config["slam"]["slam_solver_name"].as<std::string>();

  return adapter;
}