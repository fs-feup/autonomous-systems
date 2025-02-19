#include "slam_config/solver_config.hpp"

SLAMSolverParameters::SLAMSolverParameters(const SLAMParameters& params) {
  this->observation_x_noise_ = params.observation_x_noise_;
  this->observation_y_noise_ = params.observation_y_noise_;
}