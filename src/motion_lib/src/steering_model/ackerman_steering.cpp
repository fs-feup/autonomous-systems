#include "motion_lib/steering_model/ackerman_steering.hpp"

AckermanSteering::AckermanSteering(common_lib::car_parameters::CarParameters car_parameters)
    : SteeringModel(car_parameters) {
  ackerman_factor_ = car_parameters.steering_parameters->ackerman_factor;
  wheelbase_ = car_parameters.wheelbase;
  track_width_ = car_parameters.track_width;
  minimum_steering_angle_ = car_parameters.steering_parameters->minimum_steering_angle;
  maximum_steering_angle_ = car_parameters.steering_parameters->maximum_steering_angle;
}

Eigen::Vector4d AckermanSteering::calculate_steering_angles(double steering_wheel_angle) const {
  if (std::abs(steering_wheel_angle) < 1e-6) {
    return Eigen::Vector4d::Zero();
  }

  double clamped_angle =
      std::clamp(steering_wheel_angle, minimum_steering_angle_, maximum_steering_angle_);

  const double tan_sw = tan(clamped_angle);

  double fl =
      atan((wheelbase_ * tan_sw) / (wheelbase_ - track_width_ * 0.5 * tan_sw * ackerman_factor_));
  double fr =
      atan((wheelbase_ * tan_sw) / (wheelbase_ + track_width_ * 0.5 * tan_sw * ackerman_factor_));

  return Eigen::Vector4d(fl, fr, 0.0, 0.0);
}