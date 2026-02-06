#include "common_lib/car_parameters/battery_parameters.hpp"

namespace common_lib::car_parameters {

BatteryParameters::BatteryParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "motion_lib/battery_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  
}
}  // namespace common_lib::car_parameters
