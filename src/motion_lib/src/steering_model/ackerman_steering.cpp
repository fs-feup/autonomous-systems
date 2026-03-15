#include "motion_lib/steering_model/ackerman_steering.hpp"

AckermanSteering::AckermanSteering(common_lib::car_parameters::CarParameters car_parameters)
    : SteeringModel(car_parameters) {
  ackerman_factor_ = car_parameters.steering_parameters->ackerman_factor;
  wheelbase_ = car_parameters.wheelbase;
  track_width_ = car_parameters.track_width;
}

Eigen::Vector4d AckermanSteering::calculate_steering_angles(double steering_wheel_angle) const {
  if (std::abs(steering_wheel_angle) < 1e-6) {
    return Eigen::Vector4d::Zero();
  }

  const double tan_sw = tan(steering_wheel_angle);

  double inner =
      atan((wheelbase_ * tan_sw) / (wheelbase_ - track_width_ * 0.5 * tan_sw * ackerman_factor_));
  double outer =
      atan((wheelbase_ * tan_sw) / (wheelbase_ + track_width_ * 0.5 * tan_sw * ackerman_factor_));

  double fl, fr;

  if (steering_wheel_angle > 0.0) {
    // Left
    fl = inner;
    fr = outer;
  } else {
    // Right
    fl = outer;
    fr = inner;
  }

  return Eigen::Vector4d(fl, fr, 0.0, 0.0);
}