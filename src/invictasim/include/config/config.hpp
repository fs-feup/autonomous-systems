#pragma once

#include <map>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "rclcpp/rclcpp.hpp"

struct InvictaSimParameters {
  // Global config parameters
  std::string discipline;
  bool use_simulated_se;
  bool use_simulated_perception;
  bool use_simulated_planning;
  bool use_simulated_velocities;

  // Simulator config parameters
  int sim_frequency;
  double sim_speed;
  std::string track_name;
  std::string input_adapter;
  std::string output_adapter;

  std::string vehicle_model;
  std::string tire_model;
  std::string aero_model;
  std::string steering_model;
  std::string steering_motor_model;
  std::string load_transfer_model;
  std::string motor_model;
  std::string battery_model;
  std::string transmission_model;

  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters;

  // Constructor will read the config files and populate the parameters
  InvictaSimParameters();
};
