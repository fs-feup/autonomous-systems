#pragma once

#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"

class AeroModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  AeroModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
  /**
   * @brief Calculate the aero forces
   *
   * @param velocity Velocity of the car in m/s in the car's body frame [vx, vy, angular_velocity]
   * @return Eigen::Vector3d Aero forces in the car's body frame [Fx, Fy, Fz] in Newtons
   */
  virtual Eigen::Vector3d aero_forces(const Eigen::Vector3d& velocity) const = 0;
};
