#pragma once

#include "motion_lib/steering_model/base_steering_model.hpp"
/**
 * @brief Steering model that assumes parallel front steering, meaning that front wheels turn as
 * much as the steering wheel angle, and rear wheels do not steer.
 *
 */
class ParallelFrontSteering : public SteeringModel {
public:
  explicit ParallelFrontSteering(common_lib::car_parameters::CarParameters car_parameters)
      : SteeringModel(car_parameters) {}
  Eigen::Vector4d calculate_steering_angles(double steering_wheel_angle) const override;
};