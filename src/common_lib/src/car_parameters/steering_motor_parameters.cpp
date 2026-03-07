#include "common_lib/car_parameters/steering_motor_parameters.hpp"

namespace common_lib::car_parameters {

SteeringMotorParameters::SteeringMotorParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/steering_motor_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["steering_motor"];
  if (config["kp"]) {
    kp = config["kp"].as<double>();
  }
  if (config["ki"]) {
    ki = config["ki"].as<double>();
  }
  if (config["kd"]) {
    kd = config["kd"].as<double>();
  }
}
}  // namespace common_lib::car_parameters
