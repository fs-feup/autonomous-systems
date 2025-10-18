#include "motion_lib/s2v_model/no_rear_wss_bicycle_model.hpp"

std::pair<double, double> NoRearWSSBicycleModel::wheels_velocities_to_cg(
    double rl_rpm, [[maybe_unused]] double fl_rpm, double rr_rpm, [[maybe_unused]] double fr_rpm,
    double steering_angle) {
  std::pair<double, double> velocities = {0, 0};
  double rl_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rl_rpm, this->car_parameters_.wheel_diameter);
  double rr_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rr_rpm, this->car_parameters_.wheel_diameter);
  if (steering_angle == 0) {  // If no steering angle, moving straight
    velocities.first = (rl_velocity + rr_velocity) / 2;
  } else if (steering_angle > 0) {
    double rear_axis_center_rotation_radius = this->car_parameters_.wheelbase / tan(steering_angle);
    velocities.second =
        rl_velocity / (rear_axis_center_rotation_radius - (this->car_parameters_.axis_length / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.rear_axis_to_camera, 2)) *
                       fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = this->car_parameters_.wheelbase / tan(steering_angle);
    velocities.second =
        rr_velocity / (rear_axis_center_rotation_radius + (this->car_parameters_.axis_length / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.rear_axis_to_camera, 2)) *
                       fabs(velocities.second);
  }
  return velocities;
}

common_lib::structures::Position NoRearWSSBicycleModel::rear_axis_position(
    const common_lib::structures::Position& cg, double orientation, double dist_cg_2_rear_axis) {
  common_lib::structures::Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * sin(orientation);
  return rear_axis;
}

Eigen::VectorXd NoRearWSSBicycleModel::cg_velocity_to_wheels(const Eigen::Vector3d& cg_velocities) {
  const double vx = cg_velocities(0);
  const double vy = cg_velocities(1);
  const double omega = cg_velocities(2);
  const double lr =
      this->car_parameters_
          .dist_cg_2_rear_axis;  // distance from the center of mass to the rear wheels
  const double lf =
      this->car_parameters_.wheelbase -
      this->car_parameters_
          .dist_cg_2_rear_axis;  // distance from the center of mass to the front wheels

  double rear_wheel_velocity = sqrt(pow(vx, 2) + pow(vy - omega * lr, 2));

  double front_wheel_velocity = sqrt(pow(vx, 2) + pow(vy + omega * lf, 2));
  double sign = (vx < 0) ? -1.0 : 1.0;
  double front_wheels_rpm =
      sign * 60 * front_wheel_velocity / (M_PI * this->car_parameters_.wheel_diameter);
  double v = sqrt(pow(vx, 2) + pow(vy, 2));
  double steering_angle =
      (std::fabs(v) <= 1e-2) ? 0 : atan(omega * (this->car_parameters_.wheelbase) / v);
  double motor_rpm = sign * 60 * this->car_parameters_.gear_ratio * rear_wheel_velocity /
                     (M_PI * this->car_parameters_.wheel_diameter);

  Eigen::VectorXd observations = Eigen::VectorXd::Zero(4);
  observations << front_wheels_rpm, front_wheels_rpm, steering_angle, motor_rpm;
  return observations;
}

Eigen::MatrixXd NoRearWSSBicycleModel::jacobian_cg_velocity_to_wheels(
    const Eigen::Vector3d& cg_velocities) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(4, 3);
  double vx = cg_velocities(0) == 0.0 ? 1e-6 : cg_velocities(0);
  double vy = cg_velocities(1) == 0.0 ? 1e-6 : cg_velocities(1);
  double omega = cg_velocities(2) == 0.0 ? 1e-6 : cg_velocities(2);
  double lr = this->car_parameters_
                  .dist_cg_2_rear_axis;  // distance from the center of mass to the rear wheels
  double lf = this->car_parameters_.wheelbase -
              this->car_parameters_
                  .dist_cg_2_rear_axis;  // distance from the center of mass to the front wheels
  double rear_wheel_velocity = sqrt(pow(vx, 2) + pow(vy - omega * lr, 2));
  double front_wheel_velocity = sqrt(pow(vx, 2) + pow(vy + omega * lf, 2));

  double sign = (vx < 0) ? -1.0 : 1.0;
  jacobian(0, 0) =
      sign * 60 * vx / (M_PI * this->car_parameters_.wheel_diameter * front_wheel_velocity);
  jacobian(0, 1) = sign * 60 * (vy + omega * lf) /
                   (M_PI * this->car_parameters_.wheel_diameter * front_wheel_velocity);
  jacobian(0, 2) = sign * 60 * lf * (vy + omega * lf) /
                   (M_PI * this->car_parameters_.wheel_diameter * front_wheel_velocity);
  jacobian(1, 0) = jacobian(0, 0);
  jacobian(1, 1) = jacobian(0, 1);
  jacobian(1, 2) = jacobian(0, 2);

  jacobian(3, 0) = sign * this->car_parameters_.gear_ratio * 60 * vx /
                   (M_PI * this->car_parameters_.wheel_diameter * rear_wheel_velocity);
  jacobian(3, 1) = sign * this->car_parameters_.gear_ratio * 60 * (vy - omega * lr) /
                   (M_PI * this->car_parameters_.wheel_diameter * rear_wheel_velocity);
  jacobian(3, 2) = sign * this->car_parameters_.gear_ratio * 60 * (-lr) * (vy - omega * lr) /
                   (M_PI * this->car_parameters_.wheel_diameter * rear_wheel_velocity);
  double L = this->car_parameters_.wheelbase;
  double v = sqrt(pow(vx, 2) + pow(vy, 2));

  jacobian(2, 0) =
      -((omega * L) / std::max(v * v + (omega * L) * (omega * L), 1e-6)) * (vx / std::max(v, 1e-6));
  jacobian(2, 1) =
      -((omega * L) / std::max(v * v + (omega * L) * (omega * L), 1e-6)) * (vy / std::max(v, 1e-6));
  jacobian(2, 2) = (L * v) / std::max(v * v + (omega * L) * (omega * L), 1e-6);

  return jacobian;
}