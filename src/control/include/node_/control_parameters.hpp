#pragma once
#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
struct ControlParameters {
  bool using_simulated_slam_;
  bool using_simulated_velocities_;
  bool use_simulated_planning_;
  double lookahead_gain_;
  double lookahead_minimum_;
  double pid_kp_;
  double pid_ki_;
  double pid_kd_;
  double pid_tau_;
  double pid_t_;
  double pid_lim_min_;
  double pid_lim_max_;
  double pid_anti_windup_;
  double lpf_alpha_;
  double lpf_initial_value_;
  double stanley_k_;
  double stanley_epsilon_;
  std::string map_frame_id_;
  std::string lat_controller;
  common_lib::car_parameters::CarParameters car_parameters_;
};