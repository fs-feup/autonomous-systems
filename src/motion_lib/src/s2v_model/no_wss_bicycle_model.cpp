#include "motion_lib/s2v_model/no_wss_bicycle_model.hpp"

std::pair<double, double> NoWSSBicycleModel::wheels_velocities_to_cg(double rl_rpm,
                                                                     [[maybe_unused]] double fl_rpm,
                                                                     double rr_rpm,
                                                                     [[maybe_unused]] double fr_rpm,
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
        rl_velocity / (rear_axis_center_rotation_radius - (this->car_parameters_.track_width / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.rear_axis_to_camera, 2)) *
                       fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = this->car_parameters_.wheelbase / tan(steering_angle);
    velocities.second =
        rr_velocity / (rear_axis_center_rotation_radius + (this->car_parameters_.track_width / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.rear_axis_to_camera, 2)) *
                       fabs(velocities.second);
  }
  return velocities;
}

common_lib::structures::Position NoWSSBicycleModel::rear_axis_position(
    const common_lib::structures::Position& cg, double orientation, double dist_cg_2_rear_axis) {
  common_lib::structures::Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * sin(orientation);
  return rear_axis;
}

Eigen::VectorXd NoWSSBicycleModel::cg_velocity_to_wheels(const Eigen::Vector3d& cg_velocities) {
  const double vx = cg_velocities(0);
  const double omega = cg_velocities(2);
  double steering_angle =
      (std::fabs(vx) <= 1e-2) ? 0 : atan(omega * (this->car_parameters_.wheelbase) / vx);
  double motor_rpm =
      60 * this->car_parameters_.gear_ratio * vx / (M_PI * this->car_parameters_.wheel_diameter);

  Eigen::VectorXd observations = Eigen::VectorXd::Zero(2);
  observations << steering_angle, motor_rpm;
  return observations;
}

Eigen::MatrixXd NoWSSBicycleModel::jacobian_cg_velocity_to_wheels(
    const Eigen::Vector3d& cg_velocities) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, 3);
  double vx = cg_velocities(0);
  double omega = cg_velocities(2);
  double common_term = (std::fabs(vx) <= 1e-2)
                           ? 0
                           : 1 / (1 + pow((this->car_parameters_.wheelbase * omega) / vx, 2));

  jacobian(0, 0) =
      common_term *
      (std::fabs(vx) <= 1e-2 ? 0 : -this->car_parameters_.wheelbase * omega / (vx * vx));
  jacobian(0, 1) = 0;
  jacobian(0, 2) = common_term * (std::fabs(vx) <= 1e-2 ? 0 : this->car_parameters_.wheelbase / vx);
  jacobian(1, 0) =
      60 * this->car_parameters_.gear_ratio / (M_PI * this->car_parameters_.wheel_diameter);
  jacobian(1, 1) = 0;
  jacobian(1, 2) = 0;
  return jacobian;
}