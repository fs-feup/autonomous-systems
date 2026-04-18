#include "common_lib/car_parameters/transmission_parameters.hpp"

namespace common_lib::car_parameters {

TransmissionParameters::TransmissionParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/transmission_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["transmission"];

  gear_ratio = config["gear_ratio"].as<double>();
  efficiency = config["efficiency"].as<double>();
  kv = config["kv"].as<double>();
  t_max = config["t_max"].as<double>();
  viscous_drag_coeff = config["viscous_drag_coeff"].as<double>();
  coulomb_drag = config["coulomb_drag"].as<double>();
}

}  // namespace common_lib::car_parameters