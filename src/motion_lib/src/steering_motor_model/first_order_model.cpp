#include "motion_lib/steering_motor_model/first_order_model.hpp"

FirstOrderSteeringMotor::FirstOrderSteeringMotor(
    const common_lib::car_parameters::CarParameters& car_parameters)
    : SteeringMotorModel(car_parameters) {
  time_constant_ = car_parameters.steering_motor_parameters->time_constant;
}

double FirstOrderSteeringMotor::compute_steering_rate(double current_steering,
                                                      double steering_goal) {
  // Calculate the error between the desired and current steering angles
  double error = steering_goal - current_steering;

  // Compute the steering rate using a first-order response
  double steering_rate = error / time_constant_;

  return steering_rate;
}