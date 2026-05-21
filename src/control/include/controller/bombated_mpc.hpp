#pragma once

#include "base_controller.hpp"
#include "solver/map.hpp"
#include "local_pather/map.hpp"

class MPC : public Controller {
  std::shared_ptr<SolverInterface> solver_;
  std::vector<double> solver_state_ = std::vector<double>(13, 0.0); // state vector for the solver
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
  std::vector<double> path_data;

  void print_debug_info();
  bool stopping_the_car(); // Checks if we're trying to fully stop the car
public:
  void limit_velocity_according_to_current(custom_interfaces::msg::PathPointArray& path_msg);
  unsigned int compute_starting_index();
  void resample_path_with_spline(custom_interfaces::msg::PathPointArray& path_msg);
  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;
  void create_local_path(custom_interfaces::msg::PathPointArray& path_msg);
  void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) override;
  common_lib::structures::ControlCommand get_control_command() override;
  void set_path_in_solver();

  MPC(const ControlParameters& params);
  virtual ~MPC() = default;
};