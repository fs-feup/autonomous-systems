#include "motion_lib/steering_motor_model/pid_steering_motor.hpp"

double PIDSteeringMotor::compute_steering_rate(double current_steering, double steering_goal) {
  if (this->last_update == rclcpp::Time(0, 0)) {
    this->last_update = rclcpp::Clock().now();
    return 0;
  }
  double dt = (rclcpp::Clock().now() - this->last_update).seconds();
  double error = steering_goal - current_steering;

  double proportional = error;

  integral_ += error * dt;

  double derivative = (error - previous_error_) / dt;
  previous_error_ = error;

  // PID control output
  double output = this->car_parameters_->steering_motor_parameters->kp * proportional +
                  this->car_parameters_->steering_motor_parameters->ki * integral_ +
                  this->car_parameters_->steering_motor_parameters->kd * derivative;

  return output;
}