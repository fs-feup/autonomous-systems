#include "motion_lib/aero_model/map_based_aero_model.hpp"

#include <cmath>
#include <iterator>

MapBasedAeroModel::MapBasedAeroModel(
    const common_lib::car_parameters::CarParameters& car_parameters)
    : AeroModel(car_parameters) {
  extract_coefficients();
}

void MapBasedAeroModel::extract_coefficients() {
  const auto& aero = car_parameters_->aero_parameters;

  if (aero->cd_map.empty() || aero->cl_map.empty()) {
    cd_ = aero->drag_coefficient;
    cl_ = aero->lift_coefficient;
    return;
  }

  cd_ = interpolate_2d(aero->cd_map, aero->ride_height_front, aero->ride_height_rear);
  cl_ = interpolate_2d(aero->cl_map, aero->ride_height_front, aero->ride_height_rear);
}

double MapBasedAeroModel::interpolate_2d(
    const std::map<double, std::map<double, double>>& table, double rhf, double rhr) const {
  if (table.empty()) {
    return 0.0;
  }

  auto rhf_upper = table.lower_bound(rhf);
  if (rhf_upper == table.end()) {
    return interpolate_1d(table.rbegin()->second, rhr);
  }
  if (rhf_upper == table.begin()) {
    return interpolate_1d(rhf_upper->second, rhr);
  }

  auto rhf_lower = std::prev(rhf_upper);
  double val_lower = interpolate_1d(rhf_lower->second, rhr);
  double val_upper = interpolate_1d(rhf_upper->second, rhr);

  double t = (rhf - rhf_lower->first) / (rhf_upper->first - rhf_lower->first);
  return val_lower + t * (val_upper - val_lower);
}

double MapBasedAeroModel::interpolate_1d(const std::map<double, double>& row, double key) const {
  if (row.empty()) {
    return 0.0;
  }

  auto upper = row.lower_bound(key);
  if (upper == row.end()) {
    return row.rbegin()->second;
  }
  if (upper == row.begin()) {
    return upper->second;
  }

  auto lower = std::prev(upper);
  double t = (key - lower->first) / (upper->first - lower->first);
  return lower->second + t * (upper->second - lower->second);
}

Eigen::Vector3d MapBasedAeroModel::aero_forces(const Eigen::Vector3d& velocity) const {
  const double vx = velocity[0];
  const double vy = velocity[1];

  const double air_density = this->car_parameters_->aero_parameters->air_density;
  const double frontal_area = this->car_parameters_->aero_parameters->frontal_area;
  const double side_force_coefficient =
      this->car_parameters_->aero_parameters->aero_side_force_coefficient;

  const double Fx = -0.5 * air_density * frontal_area * cd_ * std::abs(vx) * vx;
  const double Fy = -0.5 * air_density * frontal_area * side_force_coefficient * std::abs(vy) * vy;
  const double Fz = -0.5 * air_density * frontal_area * cl_ * vx * vx;

  return Eigen::Vector3d(Fx, Fy, Fz);
}
