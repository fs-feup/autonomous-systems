#pragma once

#include "base_local_pather.hpp"

/**
 * @brief Base (abstract) class for local path creation, which takes the global path from the planner and the current vehicle state, and produces a local path that is sent to the controller
 */
class Interpolator : public LocalPather {
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  virtual void create_local_path(custom_interfaces::msg::PathPointArray& msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) override;

  Interpolator(const ControlParameters& params) : LocalPather(params) {};
};
