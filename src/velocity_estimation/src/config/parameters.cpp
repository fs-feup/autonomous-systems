#include "config/parameters.hpp"

std::string VEParameters::load_config() {
  std::string global_config_path = common_lib::config_load::get_config_yaml_path(
      "velocity_estimation", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string adapter = global_config["global"]["adapter"].as<std::string>();

  std::string ve_config_path = common_lib::config_load::get_config_yaml_path(
      "velocity_estimation", "velocity_estimation", adapter);

  YAML::Node ve_config = YAML::LoadFile(ve_config_path);

  this->_estimation_method_ =
      ve_config["velocity_estimation"]["estimation_method"].as<std::string>();
  this->imu_acceleration_noise_ =
      ve_config["velocity_estimation"]["imu_acceleration_noise"].as<double>();
  this->imu_rotational_noise_ =
      ve_config["velocity_estimation"]["imu_rotational_noise"].as<double>();
  this->wheel_speed_noise_ = ve_config["velocity_estimation"]["wheel_speed_noise"].as<double>();
  this->steering_angle_noise_ =
      ve_config["velocity_estimation"]["steering_angle_noise"].as<double>();
  this->motor_rpm_noise_ = ve_config["velocity_estimation"]["motor_rpm_noise"].as<double>();
  this->_wheel_base_ = ve_config["velocity_estimation"]["wheel_base"].as<double>();
  this->_weight_distribution_front_ =
      ve_config["velocity_estimation"]["weight_distribution_front"].as<double>();
  this->_gear_ratio_ = ve_config["velocity_estimation"]["gear_ratio"].as<double>();
  this->_wheel_radius_ = ve_config["velocity_estimation"]["wheel_radius"].as<double>();

  return adapter;
}