#include "common_lib/car_parameters/load_transfer_parameters.hpp"

namespace common_lib::car_parameters {

LoadTransferParameters::LoadTransferParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "car/load_transfer_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["load_transfer"];

  roll_axis_z = config["roll_axis_z"].as<double>();
  front_roll_center_z = config["front_roll_center_z"].as<double>();
  rear_roll_center_z = config["rear_roll_center_z"].as<double>();
  front_stiffness_distribution = config["front_stiffness_distribution"].as<double>();
  pitch_center_z = config["pitch_center_z"].as<double>();
  front_wheel_rate = config["front_wheel_rate"].as<double>();
  rear_wheel_rate = config["rear_wheel_rate"].as<double>();
 front_wheel_rate = config["front_wheel_rate"].as<double>();
  rear_wheel_rate = config["rear_wheel_rate"].as<double>();
}
}  // namespace common_lib::car_parameters
