#include "perception_sensor_lib/observation_model/ve/no_slip_bicycle_model.hpp"

Eigen::VectorXd NoSlipBicycleModel::expected_observations(
    const Eigen::VectorXd& cg_velocities) const {
  const double vx = cg_velocities(0);
  const double vy = cg_velocities(1);
  const double omega = cg_velocities(2);
  const double lr =
      this->car_parameters_
          ->dist_cg_2_rear_axis;  // distance from the center of mass to the rear wheels
  const double lf =
      this->car_parameters_->wheelbase -
      this->car_parameters_
          ->dist_cg_2_rear_axis;  // distance from the center of mass to the front wheels

  double rear_wheel_velocity = sqrt(pow(vx, 2) + pow(vy - omega * lr, 2));
  double rear_wheels_rpm =
      60 * rear_wheel_velocity / (M_PI * this->car_parameters_->wheel_diameter);
  if (vx < 0) {
    rear_wheels_rpm = -rear_wheels_rpm;
  }

  double front_wheel_velocity = sqrt(pow(vx, 2) + pow(vy + omega * lf, 2));
  double front_wheels_rpm =
      60 * front_wheel_velocity / (M_PI * this->car_parameters_->wheel_diameter);
  if (vx < 0) {
    front_wheels_rpm = -front_wheels_rpm;
  }
  double v = sqrt(pow(vx, 2) + pow(vy, 2));
  double steering_angle =
      (std::fabs(v) <= 1e-2) ? 0 : atan(omega * (this->car_parameters_->wheelbase) / v);
  double motor_rpm = 60 * this->car_parameters_->gear_ratio * rear_wheel_velocity /
                     (M_PI * this->car_parameters_->wheel_diameter);
  if (vx < 0) {
    motor_rpm = -motor_rpm;
  }

  Eigen::VectorXd observations = Eigen::VectorXd::Zero(6);
  observations << front_wheels_rpm, front_wheels_rpm, rear_wheels_rpm, rear_wheels_rpm,
      steering_angle, motor_rpm;
  return observations;
}

Eigen::MatrixXd NoSlipBicycleModel::expected_observations_jacobian(
    const Eigen::VectorXd& cg_velocities) const {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 3);
  double vx = cg_velocities(0);
  double vy = cg_velocities(1);
  double omega = cg_velocities(2);
  double lr = this->car_parameters_
                  ->dist_cg_2_rear_axis;  // distance from the center of mass to the rear wheels
  double lf = this->car_parameters_->wheelbase -
              this->car_parameters_
                  ->dist_cg_2_rear_axis;  // distance from the center of mass to the front wheels
  double rear_wheel_velocity = sqrt(pow(vx, 2) + pow(vy - omega * lr, 2));
  double front_wheel_velocity = sqrt(pow(vx, 2) + pow(vy + omega * lf, 2));

  const double epsilon = 1e-2;
  if (std::fabs(rear_wheel_velocity) < epsilon || std::fabs(front_wheel_velocity) < epsilon) {
    jacobian(5, 0) = epsilon;
    return jacobian;
  }

  double sign = (vx < 0) ? -1.0 : 1.0;
  jacobian(0, 0) = std::max(
      0.0, sign * 60 * vx / (M_PI * this->car_parameters_->wheel_diameter * front_wheel_velocity));
  jacobian(0, 1) = sign * 60 * (vy + omega * lf) /
                   (M_PI * this->car_parameters_->wheel_diameter * front_wheel_velocity);
  jacobian(0, 2) = sign * 60 * lf * (vy + omega * lf) /
                   (M_PI * this->car_parameters_->wheel_diameter * front_wheel_velocity);
  jacobian(1, 0) = jacobian(0, 0);
  jacobian(1, 1) = jacobian(0, 1);
  jacobian(1, 2) = jacobian(0, 2);

  jacobian(2, 0) =
      sign * 60 * vx / (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  jacobian(2, 1) = sign * 60 * (vy - omega * lr) /
                   (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  jacobian(2, 2) = sign * 60 * (-lr) * (vy - omega * lr) /
                   (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  jacobian(3, 0) = jacobian(2, 0);
  jacobian(3, 1) = jacobian(2, 1);
  jacobian(3, 2) = jacobian(2, 2);

  jacobian(5, 0) = sign * this->car_parameters_->gear_ratio * 60 * vx /
                   (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  jacobian(5, 1) = sign * this->car_parameters_->gear_ratio * 60 * (vy - omega * lr) /
                   (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  jacobian(5, 2) = sign * this->car_parameters_->gear_ratio * 60 * (-lr) * (vy - omega * lr) /
                   (M_PI * this->car_parameters_->wheel_diameter * rear_wheel_velocity);
  double L = this->car_parameters_->wheelbase;
  double v = sqrt(pow(vx, 2) + pow(vy, 2));
  if (std::fabs(v) > epsilon) {
    jacobian(4, 0) = -((omega * L) / (v * v + (omega * L) * (omega * L))) * (vx / v);
    jacobian(4, 1) = -((omega * L) / (v * v + (omega * L) * (omega * L))) * (vy / v);
    jacobian(4, 2) = (L * v) / (v * v + (omega * L) * (omega * L));
  }

  return jacobian;
}