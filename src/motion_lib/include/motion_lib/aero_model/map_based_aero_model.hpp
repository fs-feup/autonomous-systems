#pragma once

#include <map>

#include "base_aero_model.hpp"

class MapBasedAeroModel : public AeroModel {
public:
  MapBasedAeroModel(const common_lib::car_parameters::CarParameters& car_parameters);

  Eigen::Vector3d aero_forces(const Eigen::Vector3d& velocity) const override;

private:
  double cd_;  // Drag coefficient extracted from the map at the configured ride height
  double cl_;  // Lift coefficient extracted from the map at the configured ride height

  // Reads ride_height_front/rear from AeroParameters and interpolates cd_/cl_.
  // Falls back to the static drag_coefficient/lift_coefficient if no map is configured.
  void extract_coefficients();

  double interpolate_2d(const std::map<double, std::map<double, double>>& table, double rhf,
                        double rhr) const;
  double interpolate_1d(const std::map<double, double>& row, double key) const;
};
