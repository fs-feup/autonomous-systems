#include "common_lib/car_parameters/brake_parameters.hpp"

namespace common_lib::car_parameters {

BrakeParameters::BrakeParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/brake_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["brake"];

  max_front_torque = config["max_front_torque"].as<double>();
  max_rear_torque = config["max_rear_torque"].as<double>();
}

}  // namespace common_lib::car_parameters
