#pragma once

#include "ve_base_observation_model.hpp"

class NoSlipBicycleModel : public VEObservationModel {
public:
  NoSlipBicycleModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : VEObservationModel(car_parameters) {}

  /**
   * @brief
   *
   * @param state velocities at the center of mass of the vehicle, in its body frame.
   * The state vector must have the following structure: [vx, vy, angular velocity].
   *
   * @return Eigen::VectorXd Expected WSS, SAS and Motor Resolver Readings with the following
   * structure: [WSS_FL, WSS_FR, WSS_RL, WSS_RR, SAS, Motor Resolver].
   * WSS and Motor resolver readings are in RPMs, SAS is in radians.
   */
  Eigen::VectorXd expected_observations(const Eigen::VectorXd& state) const override;

  /**
   * @brief calculates the Jacobian of the expected_observations function with respect to the state.
   *
   * @param state velocities at the CG
   * @return Eigen::MatrixXd of size (6, 3) where 6 is the number of expected observations
   */
  Eigen::MatrixXd expected_observations_jacobian(const Eigen::VectorXd& state) const override;
};