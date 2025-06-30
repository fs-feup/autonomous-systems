#include "motion_lib/aero_model/default_aero_model.hpp"

Eigen::Vector3d DefaultAeroModel::aero_forces(const Eigen::Vector3d& velocity) const {
  return Eigen::Vector3d::Zero();
}