#include "kalman_filter/motion_models.hpp"
#include "utils/formulas.hpp"

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
    const Eigen::VectorXf& expected_state,
    const MotionPredictionData& motion_prediction_data, const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    jacobian(0, 2) = -motion_prediction_data.translational_velocity *
                     sin(expected_state(2)) * time_interval;
    jacobian(1, 2) = motion_prediction_data.translational_velocity *
                     cos(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    jacobian(0, 2) = -(motion_prediction_data.translational_velocity /
                       motion_prediction_data.rotational_velocity) *
                         cos(expected_state(2)) +
                     (motion_prediction_data.translational_velocity /
                      motion_prediction_data.rotational_velocity) *
                         cos(expected_state(2) +
                             motion_prediction_data.rotational_velocity * time_interval);
    jacobian(1, 2) = -(motion_prediction_data.translational_velocity /
                       motion_prediction_data.rotational_velocity) *
                         sin(expected_state(2)) +
                     (motion_prediction_data.translational_velocity /
                      motion_prediction_data.rotational_velocity) *
                         sin(expected_state(2) +
                             motion_prediction_data.rotational_velocity * time_interval);
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

// TODO(marhcouto): check what to do about the unused paramter warnings
Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf& expected_state,
    const MotionPredictionData& motion_prediction_data,
    const double time_interval)
    const {  // In this implementation, as the motion model is already linear,
             // we do not use the derivative of the model
  Eigen::MatrixXf motion_to_state_matrix =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  return motion_to_state_matrix;
}