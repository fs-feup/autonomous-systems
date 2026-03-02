#include "common_lib/car_parameters/differential_parameters.hpp"

namespace common_lib::car_parameters {

DifferentialParameters::DifferentialParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/differential_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["differential"];
  kv = config["kv"].as<double>();
  t_max = config["t_max"].as<double>();
  efficiency = config["efficiency"].as<double>();
}
}  // namespace common_lib::car_parameters
