#include "observation_lib/bicycle_model.hpp"

namespace observation_lib::bicycle_model {

Eigen::VectorXd estimate_observations(Eigen::Vector3d& state, double wheel_base,
                                      double weight_distribution_front, double gear_ratio,
                                      double wheel_radius) {
  double Lr = wheel_base *
              weight_distribution_front;  // distance from the center of mass to the rear wheels
  double Lf =
      wheel_base *
      (1 - weight_distribution_front);  // distance from the center of mass to the front wheels
  double rear_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) - state(2) * Lr, 2));
  double rear_wheels_rpm = 60 * rear_wheel_velocity / (2 * M_PI * wheel_radius);
  double front_wheel_velocity = sqrt(pow(state(0), 2) + pow(state(1) + state(2) * Lf, 2));
  double front_wheels_rpm = 60 * front_wheel_velocity / (2 * M_PI * wheel_radius);
  double steering_angle =
      (std::fabs(state(0)) <= 0.01) ? 0 : atan((state(1) + state(2) * Lf) / state(0));
  double motor_rpm = 60 * gear_ratio * rear_wheel_velocity / (2 * M_PI * wheel_radius);

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
  jacobian(0, 0) = 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(0, 1) =
      60 * 2 * (state(1) + state(2) * Lf) / (2 * M_PI * wheel_radius * 2 * front_wheel_velocity);
  jacobian(0, 2) = 60 * 2 * Lf * (state(1) + state(2) * Lf) /
                   (2 * M_PI * wheel_radius * 2 * front_wheel_velocity);
  jacobian(1, 0) = jacobian(0, 0);
  jacobian(1, 1) = jacobian(0, 1);
  jacobian(1, 2) = jacobian(0, 2);

  jacobian(2, 0) = 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(2, 1) =
      60 * 2 * (state(1) - state(2) * Lr) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(2, 2) = 60 * 2 * (-Lr) * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(3, 0) = jacobian(2, 0);
  jacobian(3, 1) = jacobian(2, 1);
  jacobian(3, 2) = jacobian(2, 2);

  jacobian(4, 0) =
      (-(state(1) + state(2) * Lf) / (vx * vx)) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));
  jacobian(4, 1) = (1 / vx) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));
  jacobian(4, 2) = (Lf / vx) / (1 + pow((state(1) + state(2) * Lf) / vx, 2));

  jacobian(5, 0) =
      gear_ratio * 60 * 2 * state(0) / (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(5, 1) = gear_ratio * 60 * 2 * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  jacobian(5, 2) = gear_ratio * 60 * 2 * (-Lr) * (state(1) - state(2) * Lr) /
                   (2 * M_PI * wheel_radius * 2 * rear_wheel_velocity);
  return jacobian;
}

}  // namespace observation_lib::bicycle_model