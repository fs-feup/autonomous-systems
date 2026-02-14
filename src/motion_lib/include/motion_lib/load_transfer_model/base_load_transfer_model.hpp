#pragma once

#include <Eigen/Dense>
#include <memory>
#include <cmath>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/physical_constants.hpp"

/**
 * @brief Base class for models that compute load transfer on a vehicle.
 *
 */
class LoadTransferModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;
  std::shared_ptr<common_lib::structures::PhysicalConstants> physical_constants_;

public:
  LoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param dynamic_state Contains the relevant dynamic state that affects the load transfer, can
   * include accelerations, euler angles, velocities, etc.
   * @return Eigen::Vector4d a vector containing the loads on the four tires in Newtons, in the
   * order: FL, FR, RL, RR.
   */
  virtual Eigen::Vector4d compute_loads(const Eigen::VectorXd& dynamic_state) const = 0;
};
