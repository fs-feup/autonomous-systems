#include "common_lib/car_parameters/inverter_parameters.hpp"

namespace common_lib::car_parameters {

InverterParameters::InverterParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/inverter_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["inverter"];
}

}  // namespace common_lib::car_parameters
