#include "common_lib/car_parameters/motor_parameters.hpp"

namespace common_lib::car_parameters {

MotorParameters::MotorParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/motor_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["motor"];
  if (config["max_peak_rpm"]) {
    max_peak_rpm = config["max_peak_rpm"].as<float>();
  }
  if (config["max_continuous_rpm"]) {
    max_continuous_rpm = config["max_continuous_rpm"].as<float>();
  }
  if (config["max_peak_current"]) {
    max_peak_current = config["max_peak_current"].as<float>();
  }
  if (config["max_continuous_current"]) {
    max_continuous_current = config["max_continuous_current"].as<float>();
  }
  if (config["max_continuous_power"]) {
    max_continuous_power = config["max_continuous_power"].as<float>();
  }
  if (config["max_peak_power"]) {
    max_peak_power = config["max_peak_power"].as<float>();
  }
  if (config["max_continous_torque"]) {
    max_continous_torque = config["max_continous_torque"].as<float>();
  }
  if (config["max_peak_torque"]) {
    max_peak_torque = config["max_peak_torque"].as<float>();
  }
  if (config["peak_duration"]) {
    peak_duration = config["peak_duration"].as<float>();
  }
  if (config["kt_constant"]) {
    kt_constant = config["kt_constant"].as<float>();
  }

  if (config["efficiency_map"]) {
    for (const auto& rpm_node : config["efficiency_map"]) {
      float rpm = rpm_node.first.as<float>();
      for (const auto& tq_node : rpm_node.second) {
        float tq = tq_node.first.as<float>();
        float eff = tq_node.second.as<float>();
        efficiency_map[rpm][tq] = eff;
      }
    }
  }
}
}  // namespace common_lib::car_parameters
