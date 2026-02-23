#include "common_lib/car_parameters/battery_parameters.hpp"

namespace common_lib::car_parameters {

BatteryParameters::BatteryParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/battery_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["battery"];
  if (config["nominal_voltage"]) nominal_voltage = config["nominal_voltage"].as<float>();
  if (config["capacity_ah"]) capacity_ah = config["capacity_ah"].as<float>();
  if (config["max_continuous_discharge_current"])
    max_continuous_discharge_current = config["max_continuous_discharge_current"].as<float>();
  if (config["max_peak_discharge_current"])
    max_peak_discharge_current = config["max_peak_discharge_current"].as<float>();
  if (config["internal_resistance"])
    internal_resistance = config["internal_resistance"].as<float>();
  if (config["peak_duration"]) peak_duration = config["peak_duration"].as<float>();
  if (config["soc_voltage_map"]) {
    for (const auto& node : config["soc_voltage_map"]) {
      float soc = node.first.as<float>();
      float voltage = node.second.as<float>();
      soc_voltage_map[soc] = voltage;
    }
  }
  if (config["min_voltage"]) min_voltage = config["min_voltage"].as<float>();
  if (config["max_voltage"]) max_voltage = config["max_voltage"].as<float>();
  if (config["min_soc"]) min_soc = config["min_soc"].as<float>();
}
}  // namespace common_lib::car_parameters
