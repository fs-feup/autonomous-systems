#pragma once

#include "base_controller.hpp"
#include "solver/map.hpp"
#include "local_pather/map.hpp"
#include "utils/path_stager.hpp"

class MPC : public Controller {
  std::shared_ptr<SolverInterface> solver_;
  custom_interfaces::msg::VehicleStateVector solver_state_ = custom_interfaces::msg::VehicleStateVector(); // state vector for the solver
  bool _path_received_ = false;
  custom_interfaces::msg::PathPointArray latest_path_;
  std::shared_ptr<LocalPather> local_pather_;

  // Debug strings
  std::string path_before_local;
  std::string local_path_debug;
  std::string current_state;
  std::string computed_command;
  std::string solver_state_over_horizon;
  std::string solver_command_over_horizon;

  // Path data for the solver
  custom_interfaces::msg::PathPointArray path_data;

  void print_debug_info();

  /// Rate-limits solver-failure dumps so a sustained failure cannot flood the log.
  bool should_report_failure();
  std::chrono::steady_clock::time_point last_failure_report_{};
  bool stopping_the_car();
  double last_commanded_steering_ = 0.0; // Checks if we're trying to fully stop the car
  void set_path_in_solver();
public:
  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;
  void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) override;
  common_lib::structures::ControlCommand get_control_command() override;

  MPC(const ControlParameters& params);
  virtual ~MPC() = default;
};