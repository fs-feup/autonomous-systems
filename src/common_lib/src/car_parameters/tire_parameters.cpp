#include "common_lib/car_parameters/tire_parameters.hpp"

namespace common_lib::car_parameters {

TireParameters::TireParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "motion_lib/tire_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  tire_B_lateral = config["tire_B_lateral"].as<double>();
  tire_C_lateral = config["tire_C_lateral"].as<double>();
  tire_D_lateral = config["tire_D_lateral"].as<double>();
  tire_E_lateral = config["tire_E_lateral"].as<double>();
  tire_B_longitudinal = config["tire_B_longitudinal"].as<double>();
  tire_C_longitudinal = config["tire_C_longitudinal"].as<double>();
  tire_D_longitudinal = config["tire_D_longitudinal"].as<double>();
  tire_E_longitudinal = config["tire_E_longitudinal"].as<double>();
}
}  // namespace common_lib::car_parameters
