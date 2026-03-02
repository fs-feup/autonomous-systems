#include "utils/parameters.hpp"

std::string SEParameters::load_config() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("state_estimation", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);
  this->adapter_ = global_config["global"]["adapter"].as<std::string>();

  std::string se_config_path = common_lib::config_load::get_config_yaml_path(
      "state_estimation", "state_estimation", this->adapter_);
  YAML::Node config = YAML::LoadFile(se_config_path);
  YAML::Node se_config = config["state_estimation"];

  this->estimation_method_ = se_config["estimation_method"].as<std::string>();
  this->process_model_name_ = se_config["process_model_name"].as<std::string>();
  this->observation_model_name_ = se_config["observation_model_name"].as<std::string>();

  this->state_estimation_freq_ = se_config["state_estimation_freq"].as<double>();

  this->load_transfer_model_name_ = se_config["load_transfer_model_name"].as<std::string>();
  this->aero_model_name_ = se_config["aero_model_name"].as<std::string>();
  this->steering_model_name_ = se_config["steering_model_name"].as<std::string>();
  // this->steering_motor_model_name_ = se_config["steering_motor_model_name"].as<std::string>();
  // this->differential_model_name_ = se_config["differential_model_name"].as<std::string>();
  this->tire_model_name_ = se_config["tire_model_name"].as<std::string>();

  this->velocity_x_process_noise_ = se_config["velocity_x_process_noise"].as<double>();
  this->velocity_y_process_noise_ = se_config["velocity_y_process_noise"].as<double>();
  this->yaw_rate_process_noise_ = se_config["yaw_rate_process_noise"].as<double>();
  this->acceleration_x_process_noise_ = se_config["acceleration_x_process_noise"].as<double>();
  this->acceleration_y_process_noise_ = se_config["acceleration_y_process_noise"].as<double>();
  this->steering_angle_process_noise_ = se_config["steering_angle_process_noise"].as<double>();
  this->wheel_speed_process_noise_ = se_config["wheel_speed_process_noise"].as<double>();

  this->imu_acceleration_x_noise_ = se_config["imu_acceleration_x_noise"].as<double>();
  this->imu_acceleration_y_noise_ = se_config["imu_acceleration_y_noise"].as<double>();
  this->imu_rotational_noise_ = se_config["imu_rotational_noise"].as<double>();
  this->wheel_speed_noise_ = se_config["wheel_speed_noise"].as<double>();
  this->motor_rpm_noise_ = se_config["motor_rpm_noise"].as<double>();
  this->steering_angle_noise_ = se_config["steering_angle_noise"].as<double>();

  // Load UKF parameters
  this->alpha_ = se_config["alpha"].as<double>();
  this->kappa_ = se_config["kappa"].as<double>();

  // Load car parameters
  this->car_parameters_ = common_lib::car_parameters::CarParameters();

  return this->adapter_;
}