#pragma once

#include <memory>

#include "filters/filter.hpp"
#include "gtest/gtest.h"
#include "lateral_controller/lateral_controller.hpp"
#include "node_/control_parameters.hpp"

/**
 * @brief Stanley Controller for lateral vehicle control.
 */
class Stanley : public LateralController {
private:
  double k_;        // Gain parameter
  double epsilon_;  // Smoothing parameter for low-speed

  /**
   * @brief Normalize an angle to the range [-pi, pi].
   * @param angle Angle in radians to normalize.
   * @return Normalized angle in radians.
   */
  double normalize_angle(double angle);

public:
  /**
   * @brief Constructor for Stanley controller.
   * @param lpf Low-pass filter for smoothing control output.
   * @param params Control parameters.
   */
  Stanley(std::shared_ptr<Filter> lpf, const ControlParameters& params);

  /**
   * @brief Compute the steering control law using the Stanley method.
   * @details It combines heading error, corresponding to the vehicle's orientation relative to the
   * path orientation, and cross-track error, which is the lateral distance from the vehicle to the
   * path.
   *
   * @param input Lateral control input struct.
   * @return Steering angle in radians.
   */
  double steering_control_law(const LateralControlInput& input) override;

  FRIEND_TEST(StanleyTestFixture, HeadingErrorZero);
  FRIEND_TEST(StanleyTestFixture, CrossTrackErrorZero);
  FRIEND_TEST(StanleyTestFixture, SteeringZeroError);
  FRIEND_TEST(StanleyTestFixture, SteeringWithHeadingErrorOnly);
  FRIEND_TEST(StanleyTestFixture, SteeringWithHeadingAndCrossTrackError);
};