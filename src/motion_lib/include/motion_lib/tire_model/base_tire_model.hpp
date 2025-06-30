#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

class TireModel {
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  TireModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
  /**
   * @brief Calculate the tire forces based on the tire model
   *
   * @param slip_angle Slip angle of the tire in radians
   * @param slip_ratio Slip ratio of the tire
   * @param load Load on the tire in Newtons
   * @return std::pair<double, double> Longitudinal and lateral forces
   */
  virtual std::pair<double, double> tire_forces(double slip_angle, double slip_ratio,
                                                double vertical_load) const = 0;
};
