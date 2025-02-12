#include "pose_predict/base_v2p_motion_model.hpp"

BaseV2PMotionModel::BaseV2PMotionModel(Eigen::Vector3f process_noise)
    : _process_noise_(process_noise) {}

Eigen::Matrix3f BaseV2PMotionModel::get_process_noise_matrix() {
  return _process_noise_.asDiagonal();
}

Eigen::Vector3f BaseV2PMotionModel::get_process_noise() { return _process_noise_; }