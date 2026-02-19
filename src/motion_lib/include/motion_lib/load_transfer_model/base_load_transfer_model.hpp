#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Struct that contains all possible inputs used for load transfer calculations.
 *
 */
struct LoadTransferInput {
  float longitudinal_acceleration;
  float lateral_acceleration;
  float downforce;
};

/**
 * @brief Struct that contains the outputs of the load transfer model for better readability.
 *
 */
struct LoadTransferOutput {
  float front_left_load;
  float front_right_load;
  float rear_left_load;
  float rear_right_load;
};

/**
 * @brief Base class for models that compute load transfer on a vehicle.
 *
 */
class LoadTransferModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  LoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param input Contains the relevant dynamic state that affects the load transfer, can
   * include accelerations, euler angles, velocities, etc.
   * @return Wheels a struct containing the loads on the four tires in Newtons
   */
  virtual common_lib::structures::Wheels compute_loads(const LoadTransferInput& input) const = 0;
};
