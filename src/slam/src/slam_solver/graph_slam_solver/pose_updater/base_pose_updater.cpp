#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"

#include "slam_solver/graph_slam_solver/utils.hpp"

PoseUpdater::PoseUpdater([[maybe_unused]] const SLAMParameters& params) {
  // Initialize the pose updater with parameters from SLAMParameters
  this->_last_pose_ = Eigen::Vector3d::Zero();
  this->_last_graphed_pose_ = Eigen::Vector3d::Zero();
  this->_received_first_motion_data_ = false;
  this->_last_pose_update_ = rclcpp::Time(0);
  this->_new_pose_from_graph_ = false;
  this->_last_pose_covariance_ = Eigen::Matrix3d::Zero();
}

PoseUpdater::PoseUpdater(const PoseUpdater& other) {
  this->_last_pose_ = other._last_pose_;
  this->_last_graphed_pose_ = other._last_graphed_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_new_pose_from_graph_ = other._new_pose_from_graph_;
  this->_last_pose_covariance_ = other._last_pose_covariance_;
  this->_received_first_motion_data_ = other._received_first_motion_data_;
}

PoseUpdater::~PoseUpdater() = default;

PoseUpdater& PoseUpdater::operator=(const PoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_last_graphed_pose_ = other._last_graphed_pose_;
  this->_last_pose_covariance_ = other._last_pose_covariance_;
  this->_new_pose_from_graph_ = other._new_pose_from_graph_;
  this->_received_first_motion_data_ = other._received_first_motion_data_;

  return *this;
}

std::shared_ptr<PoseUpdater> PoseUpdater::clone() const {
  return std::make_shared<PoseUpdater>(*this);
}

void PoseUpdater::update_pose(const Eigen::Vector3d& last_pose) {
  this->_last_pose_ = last_pose;
  this->_last_graphed_pose_ = last_pose;
  this->_last_pose_covariance_ = Eigen::Matrix3d::Zero();
  this->_new_pose_from_graph_ = false;  // Reset the flag for new pose from graph
}

void PoseUpdater::predict_pose(const MotionData& motion_data,
                               std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_motion_data_) {
    this->_last_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->_last_pose_update_ = motion_data.timestamp_;
    this->_received_first_motion_data_ = true;
    return;
  }
  this->_last_pose_update_.seconds(), motion_data.timestamp_.seconds();
  double delta = (motion_data.timestamp_ - this->_last_pose_update_).seconds();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "Delta time: %f", delta);
  Eigen::Vector3d new_pose =
      motion_model->get_next_pose(this->_last_pose_, *(motion_data.motion_data_), delta);
  Eigen::MatrixXd jacobian_motion_data =
      motion_model->get_jacobian_motion_data(this->_last_pose_, *(motion_data.motion_data_), delta);
  Eigen::MatrixXd motion_noise = jacobian_motion_data *
                                 motion_data.motion_data_noise_->asDiagonal() *
                                 jacobian_motion_data.transpose();
  Eigen::Vector3d pose_difference = pose_difference_eigen(this->_last_pose_, new_pose);
  Eigen::Matrix3d adjoint_matrix =
      this->get_adjoint_operator_matrix(pose_difference(0), pose_difference(1), pose_difference(2));

  this->_last_pose_ = new_pose;
  this->_last_pose_covariance_ =
      adjoint_matrix * this->_last_pose_covariance_ * adjoint_matrix.transpose() + motion_noise;
  this->_last_pose_update_ = motion_data.timestamp_;
  this->_new_pose_from_graph_ = true;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Pose Difference Covariance: \n"
                                                      << this->_last_pose_covariance_);
}

bool PoseUpdater::pose_ready_for_graph_update() const { return _new_pose_from_graph_; }

Eigen::Matrix3d PoseUpdater::get_adjoint_operator_matrix(const double x_translation,
                                                         const double y_translation,
                                                         const double rotation_angle) const {
  Eigen::Matrix3d adjoint_matrix = Eigen::Matrix3d::Zero();
  adjoint_matrix(0, 0) = cos(rotation_angle);
  adjoint_matrix(0, 1) = -sin(rotation_angle);
  adjoint_matrix(0, 2) = y_translation;
  adjoint_matrix(1, 0) = sin(rotation_angle);
  adjoint_matrix(1, 1) = cos(rotation_angle);
  adjoint_matrix(1, 2) = -x_translation;
  adjoint_matrix(2, 2) = 1.0;
  return adjoint_matrix;
}