#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/control_command.hpp"
#include "utils/state_define.hpp"

/**
 * @brief Interface for process models
 *
 * This class defines the interface for process models. Process models are used to
 * predict the next state of the vehicle based on the current state and control commands.
 */
class ProcessModel {
public:
  virtual void predict(State& state, Eigen::Matrix<double, 10, 10>& covariance,
                       const Eigen::Matrix<double, 10, 10>& process_noise_matrix,
                       common_lib::structures::ControlCommand control_command, double dt) = 0;

  virtual ~ProcessModel() = default;
};
