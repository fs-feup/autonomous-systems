#include "common_lib/car_parameters/car_parameters.hpp"

common_lib::car_parameters::CarParameters::CarParameters() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);
  std::string car_config = global_config["global"]["car_config"].as<std::string>();

  std::string car_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car", car_config);
  YAML::Node config = YAML::LoadFile(car_config_path);

  this->wheel_diameter = config["car"]["wheel_diameter"].as<double>();
  this->wheelbase = config["car"]["wheel_base"].as<double>();
  this->track_width = config["car"]["track_width"].as<double>();
  this->cg_2_rear_axis = config["car"]["cg_2_rear_axis"].as<double>();
  this->gear_ratio = config["car"]["gear_ratio"].as<double>();
  this->sprung_mass = config["car"]["sprung_mass"].as<double>();
  this->unsprung_mass = config["car"]["unsprung_mass"].as<double>();
  this->total_mass = config["car"]["total_mass"].as<double>();
  this->sprung_cg_y = config["car"]["sprung_cg_y"].as<double>();
  this->sprung_cg_z = config["car"]["sprung_cg_z"].as<double>();
  this->unsprung_cg_y = config["car"]["unsprung_cg_y"].as<double>();
  this->unsprung_cg_z = config["car"]["unsprung_cg_z"].as<double>();
  this->cg_height = config["car"]["cg_height"].as<double>();
  this->Izz = config["car"]["Izz"].as<double>();
  this->ackerman_factor = config["car"]["ackerman_factor"].as<double>();
}

common_lib::car_parameters::CarParameters::CarParameters(std::string dir, std::string config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", dir, config_name);
  YAML::Node config = YAML::LoadFile(config_path);

  std::string car_config_name;
  if (config["car_config"]) {
    car_config_name = config["car_config"].as<std::string>();
  } else {
    std::string global_config_path =
        common_lib::config_load::get_config_yaml_path("common_lib", "global", "global_config");
    YAML::Node global_config = YAML::LoadFile(global_config_path);
    car_config_name = global_config["global"]["car_config"].as<std::string>();
  }

  std::string car_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car", car_config_name);
  YAML::Node car_config = YAML::LoadFile(car_config_path);

  this->wheel_diameter = car_config["car"]["wheel_diameter"].as<double>();
  this->wheelbase = car_config["car"]["wheel_base"].as<double>();
  this->track_width = car_config["car"]["track_width"].as<double>();
  this->cg_2_rear_axis = car_config["car"]["cg_2_rear_axis"].as<double>();
  this->gear_ratio = car_config["car"]["gear_ratio"].as<double>();
  this->sprung_mass = car_config["car"]["sprung_mass"].as<double>();
  this->unsprung_mass = car_config["car"]["unsprung_mass"].as<double>();
  this->total_mass = car_config["car"]["total_mass"].as<double>();
  this->sprung_cg_y = car_config["car"]["sprung_cg_y"].as<double>();
  this->sprung_cg_z = car_config["car"]["sprung_cg_z"].as<double>();
  this->unsprung_cg_y = car_config["car"]["unsprung_cg_y"].as<double>();
  this->unsprung_cg_z = car_config["car"]["unsprung_cg_z"].as<double>();
  this->cg_height = car_config["car"]["cg_height"].as<double>();
  this->Izz = car_config["car"]["Izz"].as<double>();

  if (config["vehicle_model"]["tire_model_params"]) {
    this->tire_parameters = std::make_shared<TireParameters>(
        config["vehicle_model"]["tire_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["aero_model_params"]) {
    this->aero_parameters = std::make_shared<AeroParameters>(
        config["vehicle_model"]["aero_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["steering_motor_model_params"]) {
    this->steering_motor_parameters = std::make_shared<SteeringMotorParameters>(
        config["vehicle_model"]["steering_motor_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["steering_model_params"]) {
    this->steering_parameters = std::make_shared<SteeringParameters>(
        config["vehicle_model"]["steering_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["load_transfer_model_params"]) {
    this->load_transfer_parameters = std::make_shared<LoadTransferParameters>(
        config["vehicle_model"]["load_transfer_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["motor_model_params"]) {
    this->motor_parameters = std::make_shared<MotorParameters>(
        config["vehicle_model"]["motor_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["battery_model_params"]) {
    this->battery_parameters = std::make_shared<BatteryParameters>(
        config["vehicle_model"]["battery_model_params"].as<std::string>());
  }
  if (config["vehicle_model"]["differential_model_params"]) {
    this->differential_parameters = std::make_shared<DifferentialParameters>(
        config["vehicle_model"]["differential_model_params"].as<std::string>());
  }
}