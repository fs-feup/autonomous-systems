#include "common_lib/car_parameters/motor_parameters.hpp"

namespace common_lib::car_parameters {

MotorParameters::MotorParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "motion_lib/motor_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  // Add assignments for all fields when defined
}
}  // namespace common_lib::car_parameters
