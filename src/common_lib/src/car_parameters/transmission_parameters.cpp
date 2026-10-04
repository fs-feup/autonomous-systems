#include "common_lib/car_parameters/transmission_parameters.hpp"

namespace common_lib::car_parameters {

TransmissionParameters::TransmissionParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/transmission_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["transmission"];

  gear_ratio = config["gear_ratio"].as<double>();
  efficiency = config["efficiency"].as<double>();
  locking_stiffness = config["locking_stiffness"].as<double>();
  viscous_drag_coeff = config["viscous_drag_coeff"].as<double>();
  coulomb_drag = config["coulomb_drag"].as<double>();
  coulomb_smooth_stiffness = config["coulomb_smooth_stiffness"].as<double>();
  preload = config["preload"].as<double>();
  drive_ramp_effect = config["drive_ramp_effect"].as<double>();
  coast_ramp_effect = config["coast_ramp_effect"].as<double>();
}

}  // namespace common_lib::car_parameters