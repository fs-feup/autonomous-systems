#include "common_lib/car_parameters/steering_parameters.hpp"

namespace common_lib::car_parameters {

SteeringParameters::SteeringParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/steering_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["steering"];
  if (config["minimum_steering_angle"]) {
    minimum_steering_angle = config["minimum_steering_angle"].as<double>();
  }
  if (config["maximum_steering_angle"]) {
    maximum_steering_angle = config["maximum_steering_angle"].as<double>();
  }
}
}  // namespace common_lib::car_parameters
