#include "config/parameters.hpp"

ControlParameters::ControlParameters(const ControlParameters &params) {
  mission_ = params.mission_;
  using_simulated_slam_ = params.using_simulated_slam_;
  using_simulated_velocities_ = params.using_simulated_velocities_;
  use_simulated_planning_ = params.use_simulated_planning_;
  lookahead_gain_ = params.lookahead_gain_;
  lookahead_minimum_ = params.lookahead_minimum_;
  first_last_max_dist_ = params.first_last_max_dist_;
  pid_kp_ = params.pid_kp_;
  pid_ki_ = params.pid_ki_;
  pid_kd_ = params.pid_kd_;
  pid_tau_ = params.pid_tau_;
  pid_t_ = params.pid_t_;
  pid_lim_min_ = params.pid_lim_min_;
  pid_lim_max_ = params.pid_lim_max_;
  pid_anti_windup_ = params.pid_anti_windup_;
  pid_max_positive_error_ = params.pid_max_positive_error_;
  pid_max_negative_error_ = params.pid_max_negative_error_;
  lpf_alpha_ = params.lpf_alpha_;
  lpf_initial_value_ = params.lpf_initial_value_;
  map_frame_id_ = params.map_frame_id_;
  command_time_interval_ = params.command_time_interval_;
}

ControlParameters &ControlParameters::operator=(const ControlParameters &other) {
  if (this != &other) {
    mission_ = other.mission_;
    using_simulated_slam_ = other.using_simulated_slam_;
    using_simulated_velocities_ = other.using_simulated_velocities_;
    use_simulated_planning_ = other.use_simulated_planning_;
    lookahead_gain_ = other.lookahead_gain_;
    lookahead_minimum_ = other.lookahead_minimum_;
    first_last_max_dist_ = other.first_last_max_dist_;
    pid_kp_ = other.pid_kp_;
    pid_ki_ = other.pid_ki_;
    pid_kd_ = other.pid_kd_;
    pid_tau_ = other.pid_tau_;
    pid_t_ = other.pid_t_;
    pid_lim_min_ = other.pid_lim_min_;
    pid_lim_max_ = other.pid_lim_max_;
    pid_anti_windup_ = other.pid_anti_windup_;
    pid_max_positive_error_ = other.pid_max_positive_error_;
    pid_max_negative_error_ = other.pid_max_negative_error_;
    lpf_alpha_ = other.lpf_alpha_;
    lpf_initial_value_ = other.lpf_initial_value_;
    map_frame_id_ = other.map_frame_id_;
    command_time_interval_ = other.command_time_interval_;
  }
  return *this;
}

std::string ControlParameters::load_config() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("control", "global", "global_config");
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Loading global config from: %s",
               global_config_path.c_str());
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string adapter = global_config["global"]["adapter"].as<std::string>();
  this->using_simulated_slam_ = global_config["global"]["use_simulated_se"].as<bool>();
  this->using_simulated_velocities_ =
      global_config["global"]["use_simulated_velocities"].as<bool>();
  this->use_simulated_planning_ = global_config["global"]["use_simulated_planning"].as<bool>();

  std::string control_path =
      common_lib::config_load::get_config_yaml_path("control", "control", adapter);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Loading control config from: %s",
               control_path.c_str());
  YAML::Node control = YAML::LoadFile(control_path);

  auto control_config = control["control"];
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Control config contents: %s",
               YAML::Dump(control_config).c_str());

  this->lookahead_gain_ = control_config["lookahead_gain"].as<double>();
  this->lookahead_minimum_ = control_config["lookahead_minimum"].as<double>();
  this->first_last_max_dist_ = control_config["first_last_max_dist"].as<double>();
  this->pid_kp_ = control_config["pid_kp"].as<double>();
  this->pid_ki_ = control_config["pid_ki"].as<double>();
  this->pid_kd_ = control_config["pid_kd"].as<double>();
  this->pid_tau_ = control_config["pid_tau"].as<double>();
  this->pid_t_ = control_config["pid_t"].as<double>();
  this->pid_lim_min_ = control_config["pid_lim_min"].as<double>();
  this->pid_lim_max_ = control_config["pid_lim_max"].as<double>();
  this->pid_anti_windup_ = control_config["pid_anti_windup"].as<double>();
  this->pid_max_positive_error_ = control_config["pid_max_positive_error"].as<double>();
  this->pid_max_negative_error_ = control_config["pid_max_negative_error"].as<double>();
  this->map_frame_id_ = adapter == "eufs" ? "base_footprint" : "map";
  this->lpf_alpha_ = control_config["lpf_alpha"].as<double>();
  this->lpf_initial_value_ = control_config["lpf_initial_value"].as<double>();
  this->command_time_interval_ = control_config["command_time_interval"].as<int>();

  return adapter;
}