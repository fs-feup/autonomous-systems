#pragma once

#include "base_control_solver.hpp"
#include "lateral_controller/base_lateral_controller.hpp"
#include "longitudinal_controller/base_longitudinal_controller.hpp"

class BaseDecoupledController : public BaseControlSolver {
  std::shared_ptr<BaseLateralController> lateral_controller_;
  std::shared_ptr<BaseLongitudinalController> longitudinal_controller_;
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;

  /**
   * @brief Called when the car state (currently just velocity) is updated
   */
  void vehicle_state_callback(const custom_interfaces::msg::Velocities::SharedPtr msg) override;

  /**
   * @brief Called when the car pose is updated by SLAM
   */
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_state_msg) override;

  /**
   * @brief Returns the control command calculated by the solver
   */
  common_lib::structures::ControlCommand get_control_command() override;

  BaseDecoupledController(const ControlParameters& params) : BaseControlSolver(params) {};
  virtual ~BaseDecoupledController() = default;
};