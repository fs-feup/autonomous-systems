#include "common_lib/car_parameters/independent_drive_parameters.hpp"

namespace common_lib::car_parameters {

IndependentDriveParameters::IndependentDriveParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/transmission_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["independent_drive"];

  gear_ratio = config["gear_ratio"].as<double>();
  efficiency = config["efficiency"].as<double>();
  viscous_drag_coeff = config["viscous_drag_coeff"].as<double>();
  coulomb_drag = config["coulomb_drag"].as<double>();
  coulomb_smooth_stiffness =
      config["coulomb_smooth_stiffness"] ? config["coulomb_smooth_stiffness"].as<double>() : 20.0;
}

}  // namespace common_lib::car_parameters
