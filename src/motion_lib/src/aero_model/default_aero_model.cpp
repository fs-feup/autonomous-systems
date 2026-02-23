#include "motion_lib/aero_model/default_aero_model.hpp"

Eigen::Vector3d DefaultAeroModel::aero_forces(const Eigen::Vector3d& velocity) const {
  const double vx = velocity[0];
  const double vy = velocity[1];

  const double air_density = this->car_parameters_->aero_parameters->air_density;    // [kg/m^3]
  const double frontal_area = this->car_parameters_->aero_parameters->frontal_area;  // [m^2]
  const double drag_coefficient = this->car_parameters_->aero_parameters->drag_coefficient;
  const double side_force_coefficient =
      this->car_parameters_->aero_parameters->aero_side_force_coefficient;
  const double lift_coefficient = this->car_parameters_->aero_parameters->lift_coefficient;
  // Drag force (opposes vx)
  const double Fx = -0.5 * air_density * frontal_area * drag_coefficient * vx * std::abs(vx);

  // Side force (opposes vy)
  const double Fy = -0.5 * air_density * frontal_area * side_force_coefficient * vy * std::abs(vy);

  // Downforce/Lift
  const double Fz = -0.5 * air_density * frontal_area * lift_coefficient * vx * vx;

  return Eigen::Vector3d(Fx, Fy, Fz);
}