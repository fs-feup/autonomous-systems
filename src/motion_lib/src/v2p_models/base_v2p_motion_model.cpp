#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"

BaseV2PMotionModel::BaseV2PMotionModel(Eigen::Vector3f base_process_noise)
    : _base_process_noise_(base_process_noise) {}

BaseV2PMotionModel::BaseV2PMotionModel() : _base_process_noise_(Eigen::Vector3f::Zero()) {}

Eigen::Matrix3f BaseV2PMotionModel::get_base_process_noise_matrix() {
  return _base_process_noise_.asDiagonal();
}

Eigen::Vector3f BaseV2PMotionModel::get_base_process_noise() { return _base_process_noise_; }