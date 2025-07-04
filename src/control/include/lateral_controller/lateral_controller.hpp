#pragma once

#include <memory>

#include "common_lib/structures/position.hpp"
#include "filters/filter.hpp"
#include "node_/control_parameters.hpp"

using namespace common_lib::structures;

/**< Maximum steering angle in rad */
constexpr double MAX_STEERING_ANGLE = 0.349066;

/**< Minimum steering angle in rad */
constexpr double MIN_STEERING_ANGLE = -0.349066;

/**< Wheel base of the vehicle in m */
constexpr double WHEEL_BASE = 1.5;

/**
 * @brief Struct to hold all necessary arguments for the steering control law.
 */
struct LateralControlInput {
  // Pure Pursuit specific inputs
  Position rear_axis;
  Position cg;
  Position lookahead_point;
  Position closest_point;
  Position next_closest_point;
  double dist_cg_2_rear_axis;
  double yaw;
  double velocity;

  // Stanley specific inputs

  // Other controllers ...
};

/**
 * @brief Abstract interface for lateral controllers.
 */
class LateralController {
public:
  /**
   * @brief Constructor for the LateralController class.
   *
   * @param lpf Shared pointer to a low-pass filter used for smoothing control outputs.
   * @param params Control parameters that define the behavior of the controller.
   */
  LateralController(std::shared_ptr<Filter> lpf, const ControlParameters& params)
      : lpf_(std::move(lpf)), control_params_(params) {}

  /**
   * @brief Compute the steering control law.
   *
   * @param input Struct containing all necessary input data for the controller.
   * @return Steering angle in radians.
   */
  virtual double steering_control_law(const LateralControlInput& input) = 0;

protected:
  std::shared_ptr<Filter> lpf_;
  ControlParameters control_params_;
};