#pragma once

#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "map.hpp"

class SteeringModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  SteeringModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
  /**
   * @brief Calculate the steering angles on each wheel based on the steering wheel angle.
   *
   * @param steering_wheel_angle The angle of the steering wheel in radians.
   * @return Eigen::Vector4d A vector containing the steering angles for each wheel:
   *         [front left, front right, rear left, rear right] in radians.
   */
  virtual Eigen::Vector4d calculate_steering_angles(double steering_wheel_angle) const = 0;
};