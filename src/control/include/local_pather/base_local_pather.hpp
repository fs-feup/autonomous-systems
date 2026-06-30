#pragma once

#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "control/include/config/parameters.hpp"

/**
 * @brief Base (abstract) class for local path creation, which takes the global path from the planner and the current vehicle state, and produces a local path that is sent to the controller
 */
class LocalPather {
protected:
  std::shared_ptr<ControlParameters> params_;
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  virtual void create_local_path(custom_interfaces::msg::PathPointArray& msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) = 0;

  LocalPather(const ControlParameters& params) : params_(std::make_shared<ControlParameters>(params)) {};
  virtual ~LocalPather() = default;
};
