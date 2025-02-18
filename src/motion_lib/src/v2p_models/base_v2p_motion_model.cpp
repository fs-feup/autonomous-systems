#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

V2PMotionModel::V2PMotionModel(Eigen::Vector3d base_process_noise)
    : _base_process_noise_(base_process_noise) {}

V2PMotionModel::V2PMotionModel() : _base_process_noise_(Eigen::Vector3d::Zero()) {}

Eigen::Matrix3d V2PMotionModel::get_base_process_noise_matrix() {
  return _base_process_noise_.asDiagonal();
}

Eigen::Vector3d V2PMotionModel::get_base_process_noise() { return _base_process_noise_; }