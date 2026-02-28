#include "common_lib/car_parameters/battery_parameters.hpp"

namespace common_lib::car_parameters {

BatteryParameters::BatteryParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/battery_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["battery"];
  if (config["cells_series"]) cells_series = config["cells_series"].as<int>();
  if (config["cells_parallel"]) cells_parallel = config["cells_parallel"].as<int>();
  if (config["capacity_ah"]) capacity_ah = config["capacity_ah"].as<float>();
  if (config["max_discharge_current"])
    max_discharge_current = config["max_discharge_current"].as<float>();
  if (config["max_charge_current"]) max_charge_current = config["max_charge_current"].as<float>();
  if (config["max_voltage"]) max_voltage = config["max_voltage"].as<float>();
  if (config["min_voltage"]) min_voltage = config["min_voltage"].as<float>();
  if (config["min_soc"]) min_soc = config["min_soc"].as<float>();

  if (config["OCV_a5"]) OCV_a5 = config["OCV_a5"].as<float>();
  if (config["OCV_a4"]) OCV_a4 = config["OCV_a4"].as<float>();
  if (config["OCV_a3"]) OCV_a3 = config["OCV_a3"].as<float>();
  if (config["OCV_a2"]) OCV_a2 = config["OCV_a2"].as<float>();
  if (config["OCV_a1"]) OCV_a1 = config["OCV_a1"].as<float>();
  if (config["OCV_a0"]) OCV_a0 = config["OCV_a0"].as<float>();

  if (config["R0_a5"]) R0_a5 = config["R0_a5"].as<float>();
  if (config["R0_a4"]) R0_a4 = config["R0_a4"].as<float>();
  if (config["R0_a3"]) R0_a3 = config["R0_a3"].as<float>();
  if (config["R0_a2"]) R0_a2 = config["R0_a2"].as<float>();
  if (config["R0_a1"]) R0_a1 = config["R0_a1"].as<float>();
  if (config["R0_a0"]) R0_a0 = config["R0_a0"].as<float>();

  if (config["R1_a5"]) R1_a5 = config["R1_a5"].as<float>();
  if (config["R1_a4"]) R1_a4 = config["R1_a4"].as<float>();
  if (config["R1_a3"]) R1_a3 = config["R1_a3"].as<float>();
  if (config["R1_a2"]) R1_a2 = config["R1_a2"].as<float>();
  if (config["R1_a1"]) R1_a1 = config["R1_a1"].as<float>();
  if (config["R1_a0"]) R1_a0 = config["R1_a0"].as<float>();

  if (config["C1_a5"]) C1_a5 = config["C1_a5"].as<float>();
  if (config["C1_a4"]) C1_a4 = config["C1_a4"].as<float>();
  if (config["C1_a3"]) C1_a3 = config["C1_a3"].as<float>();
  if (config["C1_a2"]) C1_a2 = config["C1_a2"].as<float>();
  if (config["C1_a1"]) C1_a1 = config["C1_a1"].as<float>();
  if (config["C1_a0"]) C1_a0 = config["C1_a0"].as<float>();
}
}  // namespace common_lib::car_parameters
