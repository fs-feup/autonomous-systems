#include "common_lib/car_parameters/motor_parameters.hpp"

namespace common_lib::car_parameters {

MotorParameters::MotorParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/motor_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["motor_model"];
  if (config["max_peak_rpm"]) max_peak_rpm = config["max_peak_rpm"].as<float>();
  if (config["max_continuous_rpm"]) max_continuous_rpm = config["max_continuous_rpm"].as<float>();
  if (config["max_continuous_current"])
    max_continuous_current = config["max_continuous_current"].as<float>();
  if (config["max_peak_current"]) max_peak_current = config["max_peak_current"].as<float>();
  if (config["peak_duration"]) peak_duration = config["peak_duration"].as<float>();

  if (config["efficiency_map"]) {
    for (const auto& rpm_node : config["efficiency_map"]) {
      float rpm = std::stof(rpm_node.first.as<std::string>());
      for (const auto& tq_node : rpm_node.second) {
        float tq = std::stof(tq_node.first.as<std::string>());
        float eff = tq_node.second.as<float>();
        efficiency_map[rpm][tq] = eff;
      }
    }
  }

  if (config["torque_speed_continuous"]) {
    for (const auto& node : config["torque_speed_continuous"]) {
      float rpm = std::stof(node.first.as<std::string>());
      float tq = node.second.as<float>();
      torque_speed_continuous[rpm] = tq;
    }
  }
  if (config["torque_speed_peak"]) {
    for (const auto& node : config["torque_speed_peak"]) {
      float rpm = std::stof(node.first.as<std::string>());
      float tq = node.second.as<float>();
      torque_speed_peak[rpm] = tq;
    }
  }
}
}  // namespace common_lib::car_parameters
