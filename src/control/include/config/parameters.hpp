#pragma once
#include <yaml-cpp/yaml.h>

#include <string>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "rclcpp/rclcpp.hpp"

struct ControlParameters {
  common_lib::car_parameters::CarParameters car_parameters_; // car parameters

  bool using_simulated_slam_;  // true: use simulated slam, false: use real slam (must run the slam node)
  bool using_simulated_velocities_;  // true: use simulated velocities, false: use real velocities (must run the velocity_estimation node)
  bool use_simulated_planning_;  // true: use simulated planning, false: use real planning (must run the planning node)
  double pure_pursuit_lookahead_gain_; // lookahead distance gain for pure pursuit, to be multiplied by the current speed
  double pure_pursuit_lookahead_minimum_; // minimum lookahead distance for pure pursuit
  double pure_pursuit_first_last_max_dist_; // maximum distance between first and last point to consider for closing the path
  double pid_kp_; // proportional gain for PID controller
  double pid_ki_; // integral gain for PID controller
  double pid_kd_; // derivative gain for PID controller
  double pid_tau_; // Derivative low pass filter time constant
  double pid_t_; // Sample time for PID controller
  double pid_lim_min_; // minimum output limit for PID controller (maximum braking)
  double pid_lim_max_; // maximum output limit for PID controller (maximum throttle)
  double pid_anti_windup_; // anti-windup parameter for PID controller, gain of integrator impact when saturated 
  double pid_max_positive_error_; // maximum positive error for PID controller (restricts acceleration)
  double pid_max_negative_error_; // maximum negative error for PID controller (restricts braking)
  double lpf_alpha_; // low-pass filter smoothing factor (0 < alpha < 1), smaller values mean more smoothing
  double lpf_initial_value_; // initial value for low-pass filter
  std::string map_frame_id_; // frame id to publish visualization markers (can be "map", ...), for example closest point and lookahead point
  uint command_time_interval_; // time interval (in ms) between command publications



  ControlParameters(const ControlParameters &params);
  ControlParameters() = default;
  ControlParameters &operator=(const ControlParameters &other);
  std::string load_config();
};
