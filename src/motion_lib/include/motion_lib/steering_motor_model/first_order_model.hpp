#pragma once

#include <rclcpp/rclcpp.hpp>

#include "motion_lib/steering_motor_model/base_steering_motor_model.hpp"

class FirstOrderSteeringMotor : public SteeringMotorModel {
private:
  double time_constant_;  // Time constant for the first-order response (in seconds)

public:
<<<<<<< HEAD
  FirstOrderSteeringMotor(const common_lib::car_parameters::CarParameters& car_parameters);

  /**
   * @brief Computes the change in steering angle based on first-order dynamics.
   *
   * @param current_steering current steering angle in radians
   * @param steering_goal desired steering angle in radians
   * @return double change in steering angle in radians
   */
  double compute_steering_rate(double current_steering, double steering_goal) override;
};
=======
  explicit FirstOrderSteeringMotor(
      const common_lib::car_parameters::CarParameters& car_parameters);

  /**
   * @brief Computes the steering rate based on first-order dynamics.
   *
   * @param current_steering current steering angle in radians
   * @param steering_goal desired steering angle in radians
   * @return double steering rate in radians per second
   */
  double compute_steering_rate(double current_steering, double steering_goal) override;
};
>>>>>>> main
