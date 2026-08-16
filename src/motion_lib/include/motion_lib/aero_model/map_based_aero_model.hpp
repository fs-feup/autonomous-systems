#pragma once

#include <map>

#include "base_aero_model.hpp"

class MapBasedAeroModel : public AeroModel {
public:
  MapBasedAeroModel(const common_lib::car_parameters::CarParameters& car_parameters);

  Eigen::Vector3d aero_forces(const Eigen::Vector3d& velocity) const override;
  void update_ride_height(double ride_height_front, double ride_height_rear) override;

private:
  double ride_height_front_;
  double ride_height_rear_;

  double get_coefficient(const std::map<double, std::map<double, double>>& table,
                        double fallback) const;
  double interpolate_2d(const std::map<double, std::map<double, double>>& table, double rhf,
                        double rhr) const;
  double interpolate_1d(const std::map<double, double>& row, double key) const;
};