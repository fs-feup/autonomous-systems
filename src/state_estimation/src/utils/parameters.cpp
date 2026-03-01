#include "utils/parameters.hpp"

std::string SEParameters::load_config() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("state_estimation", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);
  this->adapter_ = global_config["global"]["adapter"].as<std::string>();

  std::string se_config_path = common_lib::config_load::get_config_yaml_path(
      "state_estimation", "state_estimation", this->adapter_);
  YAML::Node config = YAML::LoadFile(se_config_path);

  this->estimation_method_ = config["estimation_method"].as<std::string>();
  this->process_model_name_ = config["process_model_name"].as<std::string>();
  this->observation_model_name_ = config["observation_model_name"].as<std::string>();

  this->state_estimation_freq_ = config["state_estimation_freq"].as<double>();

  this->load_transfer_model_name_ = config["load_transfer_model_name"].as<std::string>();
  this->aero_model_name_ = config["aero_model_name"].as<std::string>();
  this->steering_model_name_ = config["steering_model_name"].as<std::string>();
  // this->steering_motor_model_name_ = config["steering_motor_model_name"].as<std::string>();
  // this->differential_model_name_ = config["differential_model_name"].as<std::string>();
  this->tire_model_name_ = config["tire_model_name"].as<std::string>();

  this->velocity_x_process_noise_ = config["velocity_x_process_noise"].as<double>();
  this->velocity_y_process_noise_ = config["velocity_y_process_noise"].as<double>();
  this->yaw_rate_process_noise_ = config["yaw_rate_process_noise"].as<double>();
  this->acceleration_x_process_noise_ = config["acceleration_x_process_noise"].as<double>();
  this->acceleration_y_process_noise_ = config["acceleration_y_process_noise"].as<double>();
  this->steering_angle_process_noise_ = config["steering_angle_process_noise"].as<double>();
  this->wheel_speed_process_noise_ = config["wheel_speed_process_noise"].as<double>();

  this->imu_acceleration_noise_ = config["imu_acceleration_noise"].as<double>();
  this->imu_rotational_noise_ = config["imu_rotational_noise"].as<double>();
  this->wheel_speed_noise_ = config["wheel_speed_noise"].as<double>();
  this->motor_rpm_noise_ = config["motor_rpm_noise"].as<double>();
  this->steering_angle_noise_ = config["steering_angle_noise"].as<double>();

  // Load UKF parameters
  this->alpha_ = config["alpha"].as<double>();
  this->kappa_ = config["kappa"].as<double>();

  // Load car parameters
  this->car_parameters_ = common_lib::car_parameters::CarParameters();

  return this->adapter_;
}