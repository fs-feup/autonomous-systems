#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "rclcpp/rclcpp.hpp"

struct InvictaSimParameters {
  // Global config parameters
  std::string discipline;

  // Simulator config parameters
  double timestep;
  std::string track_name;
  double simulation_speed;

  std::string vehicle_model;
  std::string tire_model;
  std::string aero_model;
  std::string steering_motor_model;
  std::string load_transfer_model;
  std::string motor_model;
  std::string battery_model;
  std::string differential_model;

  common_lib::car_parameters::CarParameters car_parameters;

  // Constructor will read the config files and populate the parameters
  InvictaSimParameters();
};
