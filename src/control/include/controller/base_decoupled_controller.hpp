#pragma once

#include "base_controller.hpp"
#include "lateral_controller/map.hpp"
#include "longitudinal_controller/map.hpp"

class DecoupledController : public Controller {
  // Controller for throttle
  std::shared_ptr<LateralController> lateral_controller_;
  // Controller for steering
  std::shared_ptr<LongitudinalController> longitudinal_controller_;
public:
  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;
  common_lib::structures::ControlCommand get_control_command() override;

  DecoupledController(const ControlParameters& params);
  virtual ~DecoupledController() = default;
};