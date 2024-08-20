#include "kalman_filter/motion_models.hpp"

#include "common_lib/maths/transformations.hpp"

MotionModel::MotionModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : _process_noise_covariance_matrix(process_noise_covariance_matrix) {}

NormalVelocityModel::NormalVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

ImuVelocityModel::ImuVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

Eigen::MatrixXf MotionModel::get_process_noise_covariance_matrix(
    const unsigned int state_size) const {
  return Eigen::MatrixXf::Identity(state_size, 6) * this->_process_noise_covariance_matrix *
         Eigen::MatrixXf::Identity(6, state_size);
}

Eigen::MatrixXf MotionModel::create_process_noise_covariance_matrix(float process_noise) {
  return Eigen::MatrixXf::Identity(6, 6) * process_noise;
}

/*------------------------Normal Velocity Model-----------------------*/

Eigen::VectorXf NormalVelocityModel::predict_expected_state(
    const Eigen::VectorXf &expected_state, const MotionUpdate &motion_prediction_data,
    const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;
  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {  // Rectilinear movement

    next_state(0) +=
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
    next_state(1) +=
        motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    double radius =
        motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity;
    double delta_x = -radius * sin(expected_state(2)) +
                     radius * sin(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);
    double delta_y = radius * cos(expected_state(2)) -
                     radius * cos(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);

    next_state(0) += delta_x;
    next_state(1) += delta_y;
  }
  next_state(3) = motion_prediction_data.translational_velocity * cos(next_state(2));
  next_state(4) = motion_prediction_data.translational_velocity * sin(next_state(2));
  next_state(5) = motion_prediction_data.rotational_velocity;
  next_state(2) = common_lib::maths::normalize_angle(
      expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);

  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data,
    [[maybe_unused]] const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {  // Rectilinear movement
    jacobian(0, 2) =
        -motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
    jacobian(1, 2) =
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    float ratio =
        motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity;
    float new_heading =
        expected_state(2) + motion_prediction_data.rotational_velocity * time_interval;

    jacobian(0, 2) = -ratio * (std::cos(expected_state(2)) - std::cos(new_heading));
    jacobian(1, 2) = -ratio * (std::sin(expected_state(2)) - std::sin(new_heading));
  }
  jacobian(3, 2) = -motion_prediction_data.translational_velocity * sin(expected_state(2));
  jacobian(4, 2) = motion_prediction_data.translational_velocity * cos(expected_state(2));
  return jacobian;
}

/*----------------------IMU Velocity Model ------------------------*/

Eigen::VectorXf ImuVelocityModel::predict_expected_state(const Eigen::VectorXf &expected_state,
                                                         const MotionUpdate &motion_prediction_data,
                                                         const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;

  next_state(0) += expected_state(3) * time_interval +
                   0.5 * motion_prediction_data.acceleration_x * pow(time_interval, 2);
  next_state(1) += expected_state(4) * time_interval +
                   0.5 * motion_prediction_data.acceleration_y * pow(time_interval, 2);
  next_state(5) = motion_prediction_data.rotational_velocity;
  next_state(2) =
      common_lib::maths::normalize_angle(expected_state(2) + next_state(5) * time_interval);
  next_state(3) += motion_prediction_data.acceleration_x * time_interval;
  next_state(4) += motion_prediction_data.acceleration_y * time_interval;

  return next_state;
}

Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data, const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  jacobian(0, 3) = time_interval;
  jacobian(1, 4) = time_interval;

  jacobian(2, 2) = time_interval;
  jacobian(2, 5) = time_interval;
  return jacobian;
}