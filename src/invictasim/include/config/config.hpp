#pragma once

#include <map>
#include <string>

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
  bool initial_go_signal;
  std::string control_mode;
  std::string track_name;
  std::string input_adapter;
  std::string output_adapter;
  bool tuning_evaluator_enabled;

  std::string vehicle_model;
  std::string car_parameters_config;
  std::string tire_model;
  std::string aero_model;
  std::string steering_model;
  std::string steering_motor_model;
  std::string load_transfer_model;
  std::string motor_model;
  std::string battery_model;
  std::string transmission_model;
  std::string inverter_model;
  std::string brake_model;

  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters;

  // Constructor will read the config files and populate the parameters
  InvictaSimParameters();
};
