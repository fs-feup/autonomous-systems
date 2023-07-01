#include "kalman_filter/motion_models.hpp"

#include <iostream>

#include "utils/formulas.hpp"

MotionModel::MotionModel(const Eigen::MatrixXf& process_noise_covariance_matrix)
    : _process_noise_covariance_matrix(process_noise_covariance_matrix) {}

NormalVelocityModel::NormalVelocityModel(const Eigen::MatrixXf& process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

ImuVelocityModel::ImuVelocityModel(const Eigen::MatrixXf& process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

OdometryModel::OdometryModel(const Eigen::MatrixXf& process_noise_covariance_matrix)
    : NormalVelocityModel(process_noise_covariance_matrix) {}

Eigen::MatrixXf MotionModel::get_process_noise_covariance_matrix(
    const unsigned int state_size) const {
  return Eigen::MatrixXf::Identity(state_size, 3) * this->_process_noise_covariance_matrix *
         Eigen::MatrixXf::Identity(3, state_size);
}

Eigen::VectorXf NormalVelocityModel::predict_expected_state(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;
  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    next_state(0) +=
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
    next_state(1) +=
        motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    next_state(0) +=
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            sin(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            sin(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    next_state(1) +=
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2)) -
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  }
  next_state(2) = normalize_angle(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);
  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    jacobian(0, 2) =
        -motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
    jacobian(1, 2) =
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    jacobian(0, 2) =
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            cos(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    jacobian(1, 2) =
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            sin(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            sin(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  }

  return jacobian;
}

Eigen::VectorXf ImuVelocityModel::predict_expected_state(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;
  next_state(0) += motion_prediction_data.translational_velocity_x * time_interval;
  next_state(1) += motion_prediction_data.translational_velocity_y * time_interval;
  next_state(2) =
      normalize_angle(next_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  return next_state;
}

// TODO(marhcouto): check what to do about the unused parameter warnings
Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {  // In this implementation, as the motion model is already
                                         // linear, we do not use the derivative of the model
  Eigen::MatrixXf motion_to_state_matrix =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  return motion_to_state_matrix;
}

double OdometryModel::get_wheel_velocity_from_rpm(const double rpm) {
  return rpm * OdometryModel::wheel_diameter * M_PI / 60;
}

MotionPredictionData OdometryModel::odometry_to_velocities_transform(
    const MotionPredictionData& motion_prediction_data) const {
  MotionPredictionData motion_prediction_data_transformed = motion_prediction_data;
  if (motion_prediction_data.steering_angle == 0) {  // If no steering angle, moving straight
    double lb_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.lb_speed);
    double rb_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.rb_speed);
    double lf_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.lf_speed);
    double rf_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.rf_speed);
    motion_prediction_data_transformed.translational_velocity =
        (lb_velocity + rb_velocity + lf_velocity + rf_velocity) / 4;
  } else if (motion_prediction_data.steering_angle > 0) {
    double lb_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.lb_speed);
    double rear_axis_center_rotation_radius =
        OdometryModel::wheelbase / tan(motion_prediction_data.steering_angle);
    motion_prediction_data_transformed.rotational_velocity =
        lb_velocity / (rear_axis_center_rotation_radius - (OdometryModel::axis_length / 2));
    motion_prediction_data_transformed.translational_velocity =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(OdometryModel::wheelbase / 2, 2)) *
        abs(motion_prediction_data_transformed.rotational_velocity);
  } else {
    double rb_velocity =
        OdometryModel::get_wheel_velocity_from_rpm(motion_prediction_data.rb_speed);
    double rear_axis_center_rotation_radius =
        OdometryModel::wheelbase / tan(motion_prediction_data.steering_angle);
    motion_prediction_data_transformed.rotational_velocity =
        rb_velocity / (rear_axis_center_rotation_radius + (OdometryModel::axis_length / 2));
    motion_prediction_data_transformed.translational_velocity =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(OdometryModel::wheelbase / 2, 2)) *
        abs(motion_prediction_data_transformed.rotational_velocity);
  }
  return motion_prediction_data_transformed;
}

Eigen::VectorXf OdometryModel::predict_expected_state(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {
  return NormalVelocityModel::predict_expected_state(
      expected_state, this->odometry_to_velocities_transform(motion_prediction_data),
      time_interval);
}

Eigen::MatrixXf OdometryModel::get_motion_to_state_matrix(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) const {
  return NormalVelocityModel::get_motion_to_state_matrix(
      expected_state, this->odometry_to_velocities_transform(motion_prediction_data),
      time_interval);
}