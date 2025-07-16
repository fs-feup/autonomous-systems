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

/**
 * @brief Struct to hold all necessary arguments for the steering control law for the various
 * lateral controllers.
 */
struct LateralControlInput {
  Position global_rear_axis_position;
  Position global_cg_position;
  Position lookahead_point;
  Position closest_point;
  Position next_closest_point;
  double yaw;
  double velocity;

  LateralControlInput(const Position& global_rear_axis_position, const Position& global_cg_position,
                      const Position& lookahead_point, const Position& closest_point,
                      const Position& next_closest_point, double yaw, double velocity)
      : global_rear_axis_position(global_rear_axis_position),
        global_cg_position(global_cg_position),
        lookahead_point(lookahead_point),
        closest_point(closest_point),
        next_closest_point(next_closest_point),
        yaw(yaw),
        velocity(velocity) {}

  LateralControlInput() = default;
};

/**
 * @brief Abstract interface for lateral controllers.
 */
class LateralController {
protected:
  std::shared_ptr<Filter> lpf_;
  ControlParameters control_params_;

public:
  /**
   * @brief Constructor for the LateralController class.
   *
   * @param lpf Shared pointer to a low-pass filter used for smoothing control outputs.
   * @param params Control parameters that define the behavior of the controller.
   */
  LateralController(std::shared_ptr<Filter> lpf, const ControlParameters& params)
      : lpf_(lpf), control_params_(params) {}

  /**
   * @brief Compute the steering control to send to the vehicle.
   *
   * @param input Struct containing all necessary input data for the controller.
   * @return Steering angle in radians.
   */
  virtual double steering_control_law(const LateralControlInput& input) = 0;
};