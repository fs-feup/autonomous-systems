#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "map.hpp"

class SteeringMotorModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  SteeringMotorModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Computes the change in steering angle based.
   *
   * @param current_steering current steering angle in radians
   * @param steering_goal desired steering angle in radians
   * @return double change in steering angle in radians
   */
  virtual double compute_steering_rate(double current_steering, double steering_goal) = 0;
};