#pragma once

#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "control/include/config/parameters.hpp"
#include "common_lib/structures/control_command.hpp"

class BaseLateralController {
  std::shared_ptr<ControlParameters> params_;
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  virtual void path_callback(const custom_interfaces::msg::PathPointArray& msg) = 0;

  /**
   * @brief Called when the car state (currently just velocity) is updated
   */
  virtual void vehicle_state_callback(const custom_interfaces::msg::Velocities::SharedPtr msg) = 0;

  /**
   * @brief Called when the car pose is updated by SLAM
   */
  virtual void vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_state_msg) = 0;

  /**
   * @brief Returns the steering command calculated by the solver
   */
  virtual double get_steering_command() = 0;

  BaseLateralController(const ControlParameters& params) : params_(std::make_shared<ControlParameters>(params)) {};
  virtual ~BaseLateralController() = default;
};
