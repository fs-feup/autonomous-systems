#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Base class for velocity estimation observation models.
 *
 */
class VEObservationModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  VEObservationModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Calculates expected observations based on the state. The state vector can contain
   * variable information about the vehicle's dynamic state, such as its velocity or acceleration.
   * The expected observations are the sensor readings that the model predicts based on the current
   * state of the vehicle, and can vary depending on the type of sensor being used.
   *
   * @param state The state vector containing information about the vehicle's dynamic state. May
   * have different sizes for different observation models.
   *
   * @return Eigen::VectorXd The vector of expected observations which can have different sizes
   * depending on the observation model.
   */
  virtual Eigen::VectorXd expected_observations(const Eigen::VectorXd& state) const = 0;

  /**
   * @brief Calculates the Jacobian of the expected observations with respect to the state.
   *
   * @param state The state vector containing information about the vehicle's dynamic state.
   *
   * @return Eigen::MatrixXd The Jacobian matrix of expected observations with respect to the state.
   */
  virtual Eigen::MatrixXd expected_observations_jacobian(const Eigen::VectorXd& state) const = 0;
};