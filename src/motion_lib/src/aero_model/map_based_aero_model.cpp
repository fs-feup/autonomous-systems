#include "motion_lib/aero_model/map_based_aero_model.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

MapBasedAeroModel::MapBasedAeroModel(
    const common_lib::car_parameters::CarParameters& car_parameters)
    : AeroModel(car_parameters) {
  ride_height_front_ = car_parameters_->aero_parameters->ride_height_front;
  ride_height_rear_ = car_parameters_->aero_parameters->ride_height_rear;
}

void MapBasedAeroModel::update_ride_height(double ride_height_front, double ride_height_rear) {
  ride_height_front_ = ride_height_front;
  ride_height_rear_ = ride_height_rear;
}

double MapBasedAeroModel::get_coefficient(
    const std::map<double, std::map<double, double>>& table, double fallback) const {
  if (table.empty()) {
    return fallback;
  }
  return interpolate_2d(table, ride_height_front_, ride_height_rear_);
}

double MapBasedAeroModel::interpolate_2d(
    const std::map<double, std::map<double, double>>& table, double rhf, double rhr) const {
  auto rhf_upper = table.lower_bound(rhf);
  if (rhf_upper == table.end()) {
    rhf_upper = std::prev(table.end());
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

  const auto& aero = car_parameters_->aero_parameters;
  const double cd = get_coefficient(aero->cd_map, aero->drag_coefficient);
  const double cl = get_coefficient(aero->cl_map, aero->lift_coefficient);

  const double air_density = aero->air_density;
  const double frontal_area = aero->frontal_area;
  const double side_force_coefficient = aero->aero_side_force_coefficient;

  const double Fx = -0.5 * air_density * frontal_area * cd * std::abs(vx) * vx;
  const double Fy = -0.5 * air_density * frontal_area * side_force_coefficient * std::abs(vy) * vy;
  const double Fz = -0.5 * air_density * frontal_area * cl * vx * vx;

  return Eigen::Vector3d(Fx, Fy, Fz);
}