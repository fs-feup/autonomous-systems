#pragma once

#include <memory>

#include "filters/filter.hpp"
#include "lateral_controller/lateral_controller.hpp"
#include "node_/control_parameters.hpp"

/**
 * @brief Stanley Controller for lateral vehicle control.
 */
class Stanley : public LateralController {
public:
  /**
   * @brief Constructor for Stanley controller.
   * @param lpf Low-pass filter for smoothing control output.
   * @param params Control parameters.
   */
  Stanley(std::shared_ptr<Filter> lpf, const ControlParameters& params);

  /**
   * @brief Compute the steering control law using the Stanley method.
   * @param input Lateral control input struct.
   * @return Steering angle in radians.
   */
  double steering_control_law(const LateralControlInput& input) override;

private:
  double stanley_k_;  // Stanley gain

  /**
   * @brief Normalize an angle to the range [-pi, pi].
   * @param angle Angle in radians to normalize.
   * @return Normalized angle in radians.
   */
  double normalize_angle(double angle);
};