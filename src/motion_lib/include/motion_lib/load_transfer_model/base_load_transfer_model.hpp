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
  double longitudinal_acceleration;
  double lateral_acceleration;
  double downforce;
};

/**
 * @brief Struct that contains the outputs of the load transfer model for better readability.
 *
 */
struct LoadTransferOutput {
  double front_left_load;
  double front_right_load;
  double rear_left_load;
  double rear_right_load;
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

  /**
   * @brief Returns the portion of longitudinal load transfer that is reacted through spring
   * compression, as opposed to unsprung inertia or anti-squat/anti-dive suspension geometry.
   * This is the term relevant for ride-height / heave estimates.
   *
   * @param longitudinal_acceleration Value of longitudinal acceleration
   * @return double The spring-reacted (elastic) component of longitudinal load transfer
   */
  virtual double calculate_elastic_longitudinal_transfer(
      double longitudinal_acceleration) const = 0;
};
