#pragma once

#include <rclcpp/rclcpp.hpp>

#include "motion_lib/steering_motor_model/base_steering_motor_model.hpp"

class PIDSteeringMotor : public SteeringMotorModel {
private:
  double integral_ = 0.0;        // Integral term accumulator
  double previous_error_ = 0.0;  // Previous error for derivative calculation
  rclcpp::Time last_update = rclcpp::Time(0, 0);

public:
  PIDSteeringMotor(const common_lib::car_parameters::CarParameters& car_parameters)
      : SteeringMotorModel(car_parameters) {}

  /**
   * @brief Computes the change in steering angle based on PID control.
   *
   * @param current_steering current steering angle in radians
   * @param steering_goal desired steering angle in radians
   * @return double change in steering angle in radians
   */
  double compute_steering_rate(double current_steering, double steering_goal) override;
};