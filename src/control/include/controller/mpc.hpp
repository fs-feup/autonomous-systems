#pragma once

#include "base_controller.hpp"
#include "solver/acados/acados.hpp"

class MPC : public Controller {
  std::shared_ptr<SolverInterface> solver_;
  std::vector<double> solver_state_ = std::vector<double>(13, 0.0); // state vector for the solver
public:
  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;
  common_lib::structures::ControlCommand get_control_command() override;

  MPC(const ControlParameters& params);
  virtual ~MPC() = default;
};