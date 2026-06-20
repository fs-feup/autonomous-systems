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

  this->state_model_noise_factor_ = se_config["state_model_noise_factor"].as<double>();
  this->kinematic_model_noise_factor_ = se_config["kinematic_model_noise_factor"].as<double>();
  this->cross_noise_factor_ = se_config["cross_noise_factor"].as<double>();

  // Load UKF parameters
  this->mean_weight_ = se_config["mean_weight"].as<double>();
  this->alpha_ = se_config["alpha"].as<double>();
  this->beta_ = se_config["beta"].as<double>();
  this->kappa_ = se_config["kappa"].as<double>();

  // Load Publishing options
  this->publish_vm_debug_info_ = se_config["publish_vm_debug_info"].as<bool>();
  this->publish_exec_times_ = se_config["publish_exec_times"].as<bool>();
  this->publish_sensor_health_ = se_config["publish_sensor_health"].as<bool>();

  // Load sensor fault-detection options
  this->sensor_fault_detection_enabled_ = se_config["sensor_fault_detection_enabled"].as<bool>();
  this->sensor_faulty_noise_factor_ = se_config["sensor_faulty_noise_factor"].as<double>();
  this->sensor_staleness_miss_tolerance_ =
      se_config["sensor_staleness_miss_tolerance"].as<double>();
  this->imu_frequency_ = se_config["imu_frequency"].as<double>();
  this->wheel_speed_frequency_ = se_config["wheel_speed_frequency"].as<double>();
  this->steering_frequency_ = se_config["steering_frequency"].as<double>();
  this->imu_accel_range_ = se_config["imu_accel_range"].as<double>();
  this->imu_accel_max_rate_of_change_ = se_config["imu_accel_max_rate_of_change"].as<double>();
  this->imu_yaw_range_ = se_config["imu_yaw_range"].as<double>();
  this->imu_yaw_max_rate_of_change_ = se_config["imu_yaw_max_rate_of_change"].as<double>();
  this->wheel_speed_range_ = se_config["wheel_speed_range"].as<double>();
  this->wheel_speed_max_rate_of_change_ = se_config["wheel_speed_max_rate_of_change"].as<double>();
  this->steering_range_ = se_config["steering_range"].as<double>();
  this->steering_max_rate_of_change_ = se_config["steering_max_rate_of_change"].as<double>();
  this->motor_frequency_ = se_config["motor_frequency"].as<double>();
  this->motor_rpm_range_ = se_config["motor_rpm_range"].as<double>();
  this->motor_rpm_max_rate_of_change_ = se_config["motor_rpm_max_rate_of_change"].as<double>();

  // Load Vehicle Model
  this->vehicle_model_name_ = se_config["vehicle_model"].as<std::string>();

  std::string vm_config_path = common_lib::config_load::get_config_yaml_path(
      "state_estimation", "state_estimation/vehicle_models", vehicle_model_name_);
  YAML::Node vm1_config = YAML::LoadFile(vm_config_path);
  YAML::Node vm_config = vm1_config["vehicle_model"];

  // Load model names
  this->load_transfer_model_name_ = vm_config["load_transfer_model"].as<std::string>();
  this->aero_model_name_ = vm_config["aero_model"].as<std::string>();
  this->steering_model_name_ = vm_config["steering_model"].as<std::string>();
  this->steering_motor_model_name_ = vm_config["steering_motor_model"].as<std::string>();
  this->transmission_model_name_ = vm_config["transmission_model"].as<std::string>();
  this->tire_model_name_ = vm_config["tire_model"].as<std::string>();

  // Load car parameters
  this->car_parameters_ = std::make_shared<common_lib::car_parameters::CarParameters>(
      "state_estimation/vehicle_models", vehicle_model_name_);

  return this->adapter_;
}