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
  this->dist_cg_2_rear_axis = config["car"]["dist_cg_2_rear_axis"].as<double>();
  this->gear_ratio = config["car"]["gear_ratio"].as<double>();
  this->cog_height = config["car"]["cog_height"].as<double>();
  this->mass = config["car"]["mass"].as<double>();
  this->powertrainEfficiency = config["car"]["powertrain_efficiency"].as<double>();
  this->Izz = config["car"]["Izz"].as<double>();
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
  this->dist_cg_2_rear_axis = car_config["car"]["dist_cg_2_rear_axis"].as<double>();
  this->gear_ratio = car_config["car"]["gear_ratio"].as<double>();
  this->cog_height = car_config["car"]["cog_height"].as<double>();
  this->mass = car_config["car"]["mass"].as<double>();
  this->powertrainEfficiency = car_config["car"]["powertrain_efficiency"].as<double>();
  this->Izz = car_config["car"]["Izz"].as<double>();

  if (config["car"]["tire_model_params"]) {
    this->tire_parameters =
        std::make_shared<TireParameters>(config["car"]["tire_model_params"].as<std::string>());
  }
  if (config["car"]["aero_model_params"]) {
    this->aero_parameters =
        std::make_shared<AeroParameters>(config["car"]["aero_model_params"].as<std::string>());
  }
  if (config["car"]["steering_motor_model_params"]) {
    this->steering_motor_parameters = std::make_shared<SteeringMotorParameters>(
        config["car"]["steering_motor_model_params"].as<std::string>());
  }
  if (config["car"]["steering_model_params"]) {
    this->steering_parameters = std::make_shared<SteeringParameters>(
        config["car"]["steering_model_params"].as<std::string>());
  }
  if (config["car"]["load_transfer_model_params"]) {
    this->load_transfer_parameters = std::make_shared<LoadTransferParameters>(
        config["car"]["load_transfer_model_params"].as<std::string>());
  }
  if (config["car"]["motor_model_params"]) {
    this->motor_parameters =
        std::make_shared<MotorParameters>(config["car"]["motor_model_params"].as<std::string>());
  }
  if (config["car"]["battery_model_params"]) {
    this->battery_parameters = std::make_shared<BatteryParameters>(
        config["car"]["battery_model_params"].as<std::string>());
  }
  if (config["car"]["differential_model_params"]) {
    this->differential_parameters = std::make_shared<DifferentialParameters>(
        config["car"]["differential_model_params"].as<std::string>());
  }
}