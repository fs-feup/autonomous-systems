#include "common_lib/car_parameters/aero_parameters.hpp"

namespace common_lib::car_parameters {

AeroParameters::AeroParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/aero_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["aero"];
  if (config["lift_coefficient"]) {
    lift_coefficient = config["lift_coefficient"].as<double>();
  }
  if (config["drag_coefficient"]) {
    drag_coefficient = config["drag_coefficient"].as<double>();
  }
  if (config["aero_side_force_coefficient"]) {
    aero_side_force_coefficient = config["aero_side_force_coefficient"].as<double>();
  }
  if (config["aero_balance_front"]) {
    aero_balance_front = config["aero_balance_front"].as<double>();
  }
  if (config["frontal_area"]) {
    frontal_area = config["frontal_area"].as<double>();
  }
  if (config["air_density"]) {
    air_density = config["air_density"].as<double>();
  }
  if (config["ride_height_front"]) {
    ride_height_front = config["ride_height_front"].as<double>();
  }
  if (config["ride_height_rear"]) {
    ride_height_rear = config["ride_height_rear"].as<double>();
  }
  if (config["cd_map"]) {
    for (const auto& rhf_node : config["cd_map"]) {
      double rhf = rhf_node.first.as<double>();
      for (const auto& rhr_node : rhf_node.second) {
        double rhr = rhr_node.first.as<double>();
        cd_map[rhf][rhr] = rhr_node.second.as<double>();
      }
    }
  }
  if (config["cl_map"]) {
    for (const auto& rhf_node : config["cl_map"]) {
      double rhf = rhf_node.first.as<double>();
      for (const auto& rhr_node : rhf_node.second) {
        double rhr = rhr_node.first.as<double>();
        cl_map[rhf][rhr] = rhr_node.second.as<double>();
      }
    }
  }
}
}  // namespace common_lib::car_parameters
