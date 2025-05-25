#include "config/parameters.hpp"

std::string VEParameters::load_config() {
  std::string global_config_path = common_lib::config_load::get_config_yaml_path(
      "velocity_estimation", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string adapter = global_config["global"]["adapter"].as<std::string>();

  std::string ve_config_path = common_lib::config_load::get_config_yaml_path(
      "velocity_estimation", "velocity_estimation", adapter);

  YAML::Node ve_config = YAML::LoadFile(ve_config_path);

  std::string car_config_par =
      common_lib::config_load::get_config_yaml_path("velocity_estimation", "car", adapter);

  YAML::Node car_config = YAML::LoadFile(car_config_par);

  this->_estimation_method_ =
      ve_config["velocity_estimation"]["estimation_method"].as<std::string>();
  this->_s2v_model_name_ = ve_config["velocity_estimation"]["s2v_model_name"].as<std::string>();
  this->_process_model_name_ =
      ve_config["velocity_estimation"]["process_model_name"].as<std::string>();
  this->imu_acceleration_noise_ =
      ve_config["velocity_estimation"]["imu_acceleration_noise"].as<double>();
  this->imu_rotational_noise_ =
      ve_config["velocity_estimation"]["imu_rotational_noise"].as<double>();
      this->angular_velocity_process_noise_ =
        ve_config["velocity_estimation"]["angular_velocity_process_noise"].as<double>();
  this->wheel_speed_noise_ = ve_config["velocity_estimation"]["wheel_speed_noise"].as<double>();
  this->steering_angle_noise_ =
      ve_config["velocity_estimation"]["steering_angle_noise"].as<double>();
  this->motor_rpm_noise_ = ve_config["velocity_estimation"]["motor_rpm_noise"].as<double>();

  this->car_parameters_.wheel_diameter =
      car_config["car"]["wheel_diameter"].as<double>();
  this->car_parameters_.wheelbase = car_config["car"]["wheel_base"].as<double>();
  this->car_parameters_.rear_axis_to_camera = car_config["car"]["rear_axis_to_camera"].as<double>();
  this->car_parameters_.axis_length = car_config["car"]["axis_length"].as<double>();

  this->car_parameters_.dist_cg_2_rear_axis = car_config["car"]["dist_cg_2_rear_axis"].as<double>();

  this->car_parameters_.gear_ratio = car_config["car"]["gear_ratio"].as<double>();

  return adapter;
}