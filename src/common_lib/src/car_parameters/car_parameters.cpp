#include "common_lib/car_parameters/car_parameters.hpp"

common_lib::car_parameters::CarParameters::CarParameters(double wheel_diameter, double wheelbase,
                                                         double rear_axis_to_camera,
                                                         double axis_length,
                                                         double dist_cg_2_rear_axis,
                                                         double gear_ratio)
    : wheel_diameter(wheel_diameter),
      wheelbase(wheelbase),
      rear_axis_to_camera(rear_axis_to_camera),
      axis_length(axis_length),
      dist_cg_2_rear_axis(dist_cg_2_rear_axis),
      gear_ratio(gear_ratio) {}

common_lib::car_parameters::CarParameters::CarParameters() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);
  std::string adapter = global_config["global"]["adapter"].as<std::string>();

  std::string car_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car", adapter);
  YAML::Node car_config = YAML::LoadFile(car_config_path);
  this->wheel_diameter = car_config["car"]["wheel_diameter"].as<double>();
  this->wheelbase = car_config["car"]["wheel_base"].as<double>();
  this->rear_axis_to_camera = car_config["car"]["rear_axis_to_camera"].as<double>();
  this->axis_length = car_config["car"]["axis_length"].as<double>();
  this->dist_cg_2_rear_axis = car_config["car"]["dist_cg_2_rear_axis"].as<double>();
  this->gear_ratio = car_config["car"]["gear_ratio"].as<double>();
}