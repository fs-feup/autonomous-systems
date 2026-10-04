#include "common_lib/car_parameters/inverter_parameters.hpp"

namespace common_lib::car_parameters {

InverterParameters::InverterParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/inverter_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["inverter"];
  efficiency = config["efficiency"].as<double>();
  max_phase_current = config["max_phase_current"].as<double>();
  acceleration_ramp_ms = config["acceleration_ramp_ms"].as<double>();
  regen_braking_ramp_ms = config["regen_braking_ramp_ms"].as<double>();
  regen_torque_fraction = config["regen_torque_fraction"].as<double>();

  for (const auto& mode : config["modes"]) {
    modes[mode.first.as<std::string>()] = mode.second.as<double>();
  }
  selected_mode = config["selected_mode"].as<std::string>();
  max_torque = modes.at(selected_mode);
}

}  // namespace common_lib::car_parameters
