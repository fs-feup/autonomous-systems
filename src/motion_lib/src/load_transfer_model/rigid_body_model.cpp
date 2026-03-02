#include "motion_lib/load_transfer_model/rigid_body_model.hpp"

common_lib::structures::Wheels RigidBodyLoadTransferModel::compute_loads(
    const LoadTransferInput& input) const {
  // Accelerations at the CG
  double longitudinal_acceleration = input.longitudinal_acceleration;
  double lateral_acceleration = input.lateral_acceleration;

  double mass = this->car_parameters_->total_mass;
  double g = 9.8065;  // m/s^2 TODO: make this a constant in the car parameters (maybe?)
  double cog_height = this->car_parameters_->cg_height;
  double wheelbase = this->car_parameters_->wheelbase;
  double front_weight_distribution =
      this->car_parameters_->cg_2_rear_axis / this->car_parameters_->wheelbase;
  double rear_weight_distribution = 1.0 - front_weight_distribution;
  double track_width = this->car_parameters_->track_width;

  // Static load per wheel
  double Fz_total = mass * g;
  double front_wheel_static_load = front_weight_distribution * Fz_total / 2.0;
  double rear_wheel_static_load = rear_weight_distribution * Fz_total / 2.0;

  // Weight transfer for each axle
  double longitudinal_transfer = (mass * longitudinal_acceleration * cog_height) / wheelbase;
  double lateral_transfer = (mass * lateral_acceleration * cog_height) / track_width;

  common_lib::structures::Wheels loads;

  loads.front_left = front_wheel_static_load - longitudinal_transfer / 2.0 - lateral_transfer / 2.0;
  loads.front_right =
      front_wheel_static_load - longitudinal_transfer / 2.0 + lateral_transfer / 2.0;
  loads.rear_left = rear_wheel_static_load + longitudinal_transfer / 2.0 - lateral_transfer / 2.0;
  loads.rear_right = rear_wheel_static_load + longitudinal_transfer / 2.0 + lateral_transfer / 2.0;

  return loads;
}