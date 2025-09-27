#pragma once
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <string>

#include "common_lib/config_load/config_load.hpp"
#include "rclcpp/rclcpp.hpp"

struct ControlParameters {
  int16_t mission_;
  bool using_simulated_slam_;
  bool using_simulated_velocities_;
  bool use_simulated_planning_;
  double lookahead_gain_;
  double lookahead_minimum_;
  double first_last_max_dist_;
  double pid_kp_;
  double pid_ki_;
  double pid_kd_;
  double pid_tau_;
  double pid_t_;
  double pid_lim_min_;
  double pid_lim_max_;
  double pid_anti_windup_;
  double pid_max_positive_error_;
  double pid_max_negative_error_;
  double lpf_alpha_;
  double lpf_initial_value_;
  std::string map_frame_id_;
  uint command_time_interval_;

  ControlParameters(const ControlParameters &params);
  ControlParameters() = default;
  ControlParameters &operator=(const ControlParameters &other);
  std::string load_config();
};
