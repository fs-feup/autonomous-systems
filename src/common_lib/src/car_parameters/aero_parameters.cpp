#include "common_lib/car_parameters/aero_parameters.hpp"

namespace common_lib::car_parameters {

AeroParameters::AeroParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "motion_lib/aero_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  lift_coefficient = config["lift_coefficient"].as<double>();
  drag_coefficient = config["drag_coefficient"].as<double>();
  aero_side_force_coefficient = config["aero_side_force_coefficient"].as<double>();
  aero_balance_front = config["aero_balance_front"].as<double>();
  frontal_area = config["frontal_area"].as<double>();
}
}  // namespace common_lib::car_parameters
