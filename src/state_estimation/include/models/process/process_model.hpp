#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "common_lib/structures/control_command.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"

/**
 * @brief Interface for process models
 *
 * This class defines the interface for process models. Process models are used to
 * predict the next state of the vehicle based on the current state, control commands and a time
 * interval.
 */
class ProcessModel {
protected:
  std::shared_ptr<SEParameters> parameters_;

public:
  ProcessModel(const std::shared_ptr<SEParameters>& parameters) : parameters_(parameters) {}

  virtual void predict(Eigen::Ref<State> state,
                       common_lib::structures::ControlCommand control_command, double dt) = 0;

  virtual ~ProcessModel() = default;
};
