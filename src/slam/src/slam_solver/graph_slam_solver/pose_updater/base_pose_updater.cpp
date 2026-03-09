#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"

#include "slam_solver/graph_slam_solver/utils.hpp"

PoseUpdater::PoseUpdater(const SLAMParameters& params)
    : _pose_buffer_(params.max_pose_history_updater) {
  _last_pose_ = Eigen::Vector3d::Zero();
  _last_graphed_pose_ = Eigen::Vector3d::Zero();
  _last_pose_covariance_ = Eigen::Matrix3d::Zero();
  _last_pose_update_ = rclcpp::Time(0);
  _pose_buffer_.push(TimedPose(_last_pose_, _last_pose_update_));
}

PoseUpdater::PoseUpdater(const PoseUpdater& other) : _pose_buffer_(other._pose_buffer_) {
  _last_pose_ = other._last_pose_;
  _last_graphed_pose_ = other._last_graphed_pose_;
  _last_pose_covariance_ = other._last_pose_covariance_;
  _last_pose_update_ = other._last_pose_update_;
  _received_first_motion_data_ = other._received_first_motion_data_;
  _new_pose_from_graph_ = other._new_pose_from_graph_;
}

PoseUpdater& PoseUpdater::operator=(const PoseUpdater& other) {
  if (this == &other) {
    return *this;
  }

  _last_pose_ = other._last_pose_;
  _last_graphed_pose_ = other._last_graphed_pose_;
  _last_pose_covariance_ = other._last_pose_covariance_;
  _last_pose_update_ = other._last_pose_update_;
  _received_first_motion_data_ = other._received_first_motion_data_;
  _new_pose_from_graph_ = other._new_pose_from_graph_;
  _pose_buffer_ = other._pose_buffer_;

  return *this;
}

PoseUpdater::~PoseUpdater() = default;

std::shared_ptr<PoseUpdater> PoseUpdater::clone() const {
  return std::make_shared<PoseUpdater>(*this);
}

Eigen::Vector3d PoseUpdater::get_pose_at_timestamp(const rclcpp::Time& timestamp) const {
  if (_pose_buffer_.size() == 0) {
    throw std::out_of_range("Pose buffer is empty");
  }

  for (size_t i = 0; i < _pose_buffer_.size(); i++) {
    const auto& curr = _pose_buffer_.from_end(i);
    if (curr.timestamp.seconds() <= timestamp.seconds()) {
      if (i == 0) {
        return curr.pose;
      }
      const auto& prev = _pose_buffer_.from_end(i - 1);
      auto diff_curr = timestamp.seconds() - curr.timestamp.seconds();
      auto diff_prev = prev.timestamp.seconds() - timestamp.seconds();
      return (diff_curr < diff_prev) ? curr.pose : prev.pose;
    }
  }

  return _pose_buffer_.from_end(_pose_buffer_.size() - 1).pose;
}

void PoseUpdater::update_pose(const Eigen::Vector3d& last_pose) {
  this->_last_pose_ = last_pose;
  this->_last_graphed_pose_ = last_pose;
  this->_last_pose_covariance_ = Eigen::Matrix3d::Zero();

  _pose_buffer_.clear();  // Clear the buffer so that old not optimized poses are not kept
  _pose_buffer_.push(TimedPose(this->_last_pose_, this->_last_pose_update_));

  this->_new_pose_from_graph_ = false;  // Reset the flag for new pose from graph
}

void PoseUpdater::predict_pose(const MotionData& motion_data,
                               std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_motion_data_) {
    this->_last_pose_ = Eigen::Vector3d::Zero();
    this->_last_pose_update_ = motion_data.timestamp_;
    this->_received_first_motion_data_ = true;
    _pose_buffer_.push(TimedPose(this->_last_pose_, this->_last_pose_update_));
    return;
  }

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
  this->_last_pose_update_ = motion_data.timestamp_;
  _pose_buffer_.push(TimedPose(this->_last_pose_, this->_last_pose_update_));
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Last Pose Covariance: \n"
  //                                                     << this->_last_pose_covariance_);
  this->_last_pose_covariance_ =
      adjoint_matrix * this->_last_pose_covariance_ * adjoint_matrix.transpose() +
      motion_noise;  // TODO: to improve further, consider including a noise for the intrinsic
                     // error of the motion model
  this->_last_pose_update_ = motion_data.timestamp_;
  this->_new_pose_from_graph_ = true;
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Transposed Jacobian Motion Data: \n"
  //                                                     << jacobian_motion_data.transpose());
  // RCLCPP_DEBUG_STREAM(
  //     rclcpp::get_logger("slam"),
  //     "Motion Data Noise Diagonal: \n"
  //         << static_cast<Eigen::Vector3d>(motion_data.motion_data_noise_->asDiagonal()));
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Adjoint Matrix: \n" << adjoint_matrix);
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Motion Noise: \n" << motion_noise);
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Jacobian Motion Data: \n"
  //                                                     << jacobian_motion_data);
  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("slam"), "Pose Difference Covariance: \n"
  //                                                     << this->_last_pose_covariance_);
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
