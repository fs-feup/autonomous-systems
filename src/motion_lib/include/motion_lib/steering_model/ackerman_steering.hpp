#pragma once

#include "motion_lib/steering_model/base_steering_model.hpp"
/**
 * @brief Steering model that assumes Ackerman steering geometry, meaning that front wheels turn at
 * different angles to ensure all wheels follow a circular path.
 *
 */
class AckermanSteering : public SteeringModel {
  double ackerman_factor_;
  double wheelbase_;
  double track_width_;

public:
  explicit AckermanSteering(common_lib::car_parameters::CarParameters car_parameters)
      : SteeringModel(car_parameters) {}
  Eigen::Vector4d calculate_steering_angles(double steering_wheel_angle) const override;
};