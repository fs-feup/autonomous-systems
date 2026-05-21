#pragma once

#include "base_local_pather.hpp"

/**
 * @brief Base (abstract) class for local path creation, which takes the global path from the planner and the current vehicle state, and produces a local path that is sent to the controller
 */
class AlphaBeta : public LocalPather {
private:
  double convergence_beta_ = 0.001;

  double error_distance(custom_interfaces::msg::PathPointArray& msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) const;
  double convergence_distance(double error_distance) const;
  unsigned int number_of_points(custom_interfaces::msg::PathPointArray& msg, double convergence_distance) const;
  double phoenician_factor(double number_of_points) const;
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  virtual void create_local_path(custom_interfaces::msg::PathPointArray& msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) override;

  AlphaBeta(const ControlParameters& params) : LocalPather(params) {};
};
