#include "kalman_filter/motion_models.hpp"

#include <iostream>

#include "utils/formulas.hpp"

MotionModel::MotionModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : _process_noise_covariance_matrix(process_noise_covariance_matrix) {}

NormalVelocityModel::NormalVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

ImuVelocityModel::ImuVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

Eigen::MatrixXf MotionModel::get_process_noise_covariance_matrix(
    const unsigned int state_size) const {
  return Eigen::MatrixXf::Identity(state_size, 5) * this->_process_noise_covariance_matrix *
         Eigen::MatrixXf::Identity(5, state_size);
}

/*------------------------Normal Velocity Model-----------------------*/

Eigen::VectorXf NormalVelocityModel::predict_expected_state(
    const Eigen::VectorXf &expected_state, const MotionUpdate &motion_prediction_data,
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
  next_state(3) = motion_prediction_data.translational_velocity * cos(next_state(2));
  next_state(4) = motion_prediction_data.translational_velocity * sin(next_state(2));
  next_state(2) = normalize_angle(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);

  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data,
    [[maybe_unused]] const double time_interval) const {
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

/*----------------------IMU Velocity Model ------------------------*/

Eigen::VectorXf ImuVelocityModel::predict_expected_state(const Eigen::VectorXf &expected_state,
                                                         const MotionUpdate &motion_prediction_data,
                                                         const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;

  // Calculate the total translational velocity
  float translational_velocity = sqrt(pow(motion_prediction_data.translational_velocity_x, 2) +
                                      pow(motion_prediction_data.translational_velocity_y, 2));
  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    next_state(0) += translational_velocity * cos(expected_state(2)) * time_interval;
    next_state(1) += translational_velocity * sin(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    next_state(0) +=
        -(translational_velocity / motion_prediction_data.rotational_velocity) *
            sin(expected_state(2)) +
        (translational_velocity / motion_prediction_data.rotational_velocity) *
            sin(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    next_state(1) +=
        (translational_velocity / motion_prediction_data.rotational_velocity) *
            cos(expected_state(2)) -
        (translational_velocity / motion_prediction_data.rotational_velocity) *
            cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  }
  next_state(2) = normalize_angle(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);
  next_state(3) = motion_prediction_data.translational_velocity_x;
  next_state(4) = motion_prediction_data.translational_velocity_y;

  return next_state;
}

Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data,
    [[maybe_unused]] const double time_interval)
    const {  // In this implementation, as the motion model is already
             // linear, we do not use the derivative of the model
  Eigen::MatrixXf motion_to_state_matrix =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  return motion_to_state_matrix;
}