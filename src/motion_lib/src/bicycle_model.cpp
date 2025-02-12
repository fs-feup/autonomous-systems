#include "motion_lib/bicycle_model.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#include "common_lib/maths/transformations.hpp"
#include "motion_lib/car_parameters.hpp"

namespace motion_lib::bicycle_model {

std::pair<double, double> odometry_to_velocities_transform(double rl_rpm,
                                                           [[maybe_unused]] double fl_rpm,
                                                           double rr_rpm,
                                                           [[maybe_unused]] double fr_rpm,
                                                           double steering_angle) {
  std::pair<double, double> velocities = {0, 0};
  double rl_velocity = common_lib::maths::get_wheel_velocity_from_rpm(rl_rpm, WHEEL_DIAMETER);
  double rr_velocity = common_lib::maths::get_wheel_velocity_from_rpm(rr_rpm, WHEEL_DIAMETER);
  if (steering_angle == 0) {  // If no steering angle, moving straight
    velocities.first = (rl_velocity + rr_velocity) / 2;
  } else if (steering_angle > 0) {
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    velocities.second = rl_velocity / (rear_axis_center_rotation_radius - (AXIS_LENGTH / 2));
    velocities.first =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    velocities.second = rr_velocity / (rear_axis_center_rotation_radius + (AXIS_LENGTH / 2));
    velocities.first =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        fabs(velocities.second);
  }
  return velocities;
}

common_lib::structures::Position cg_2_rear_axis(common_lib::structures::Position cg,
                                                double orientation, double dist_cg_2_rear_axis) {
  common_lib::structures::Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * sin(orientation);
  return rear_axis;
}

Eigen::VectorXd estimate_observations(Eigen::Vector3d& state, double wheel_base,
                                      double weight_distribution_front, double gear_ratio,
                                      double wheel_radius) {
  double lr = wheel_base *
              weight_distribution_front;  // distance from the center of mass to the rear wheels
  double lf =
      wheel_base *
      (1 - weight_distribution_front);  // distance from the center of mass to the front wheels

  double rear_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) - state(2) * lr, 2));
  double rear_wheels_rpm = 60 * rear_wheel_velocity / (2 * M_PI * wheel_radius);
  if (state(0) < 0) {
    rear_wheels_rpm = -rear_wheels_rpm;
  }

  double front_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) + state(2) * lf, 2));
  double front_wheels_rpm = 60 * front_wheel_velocity / (2 * M_PI * wheel_radius);
  if (state(0) < 0) {
    front_wheels_rpm = -front_wheels_rpm;
  }

  double steering_angle =
      (std::fabs(state(0)) <= 0.01) ? 0 : atan((state(1) + state(2) * lf) / state(0));
  double motor_rpm = 60 * gear_ratio * rear_wheel_velocity / (2 * M_PI * wheel_radius);
  if (state(0) < 0) {
    motor_rpm = -motor_rpm;
  }

  Eigen::VectorXd observations = Eigen::VectorXd::Zero(6);
  observations << front_wheels_rpm, front_wheels_rpm, rear_wheels_rpm, rear_wheels_rpm,
      steering_angle, motor_rpm;
  return observations;
}

Eigen::MatrixXd jacobian_of_observation_estimation(Eigen::Vector3d& state, double wheel_base,
                                                   double weight_distribution_front,
                                                   double gear_ratio, double wheel_radius) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 3);
  double Lr = wheel_base *
              weight_distribution_front;  // distance from the center of mass to the rear wheels
  double Lf =
      wheel_base *
      (1 - weight_distribution_front);  // distance from the center of mass to the front wheels
  double rear_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) - state(2) * Lr, 2));
  if (rear_wheel_velocity == 0) {
    rear_wheel_velocity = 0.00001;
  }
  double front_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) + state(2) * Lf, 2));
  if (front_wheel_velocity == 0) {
    front_wheel_velocity = 0.00001;
  }
  double vx = (std::fabs(state(0)) <= 0.001) ? 0.001 : state(0);
  if (state(0) < 0 && vx > 0) {
    vx *= -1.0;
  }
  double sign = (state(0) < 0) ? -1.0 : 1.0;
  jacobian(0, 0) = sign * 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(0, 1) = sign * 60 * 2 * (state(1) + state(2) * Lf) /
                   (2 * M_PI * wheel_radius * 2 * front_wheel_velocity);
  jacobian(0, 2) = sign * 60 * 2 * Lf * (state(1) + state(2) * Lf) /
                   (2 * M_PI * wheel_radius * 2 * front_wheel_velocity);
  jacobian(1, 0) = jacobian(0, 0);
  jacobian(1, 1) = jacobian(0, 1);
  jacobian(1, 2) = jacobian(0, 2);

  jacobian(2, 0) = sign * 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(2, 1) = sign * 60 * 2 * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(2, 2) = sign * 60 * 2 * (-Lr) * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(3, 0) = jacobian(2, 0);
  jacobian(3, 1) = jacobian(2, 1);
  jacobian(3, 2) = jacobian(2, 2);

  jacobian(4, 0) =
      (-(state(1) + state(2) * Lf) / (vx * vx)) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));
  jacobian(4, 1) = (1 / vx) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));
  jacobian(4, 2) = (Lf / vx) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));

  jacobian(5, 0) =
      sign * gear_ratio * 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(5, 1) = sign * gear_ratio * 60 * 2 * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(5, 2) = sign * gear_ratio * 60 * 2 * (-Lr) * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  return jacobian;
}

}  // namespace motion_lib::bicycle_model