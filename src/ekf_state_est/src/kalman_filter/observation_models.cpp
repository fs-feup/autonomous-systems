#include "kalman_filter/observation_models.hpp"

#include "rclcpp/rclcpp.hpp"

ObservationModel::ObservationModel(const Eigen::MatrixXf &observation_noise_covariance_matrix)
    : _observation_noise_covariance_matrix_(observation_noise_covariance_matrix) {}

Eigen::Vector2f ObservationModel::inverse_observation_model(
    const Eigen::VectorXf &expected_state, const ObservationData &observation_data) const {
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"),
               "Observation Model - Expected State: %f %f %f %f %f", expected_state(0),
               expected_state(1), expected_state(2), expected_state(3), expected_state(4));
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "Observation Model - Observation Data: (%f,%f)",
               observation_data.position.x, observation_data.position.y);

  Eigen::Matrix3f transformation_matrix = Eigen::Matrix3f::Identity();
  transformation_matrix(0, 0) = cos(expected_state(2));
  transformation_matrix(0, 1) = -sin(expected_state(2));
  transformation_matrix(0, 2) = expected_state(0);
  transformation_matrix(1, 0) = sin(expected_state(2));
  transformation_matrix(1, 1) = cos(expected_state(2));
  transformation_matrix(1, 2) = expected_state(1);

  Eigen::Vector3f observation =
      Eigen::Vector3f(observation_data.position.x, observation_data.position.y, 1);
  Eigen::Vector3f observed_landmark_absolute_position = transformation_matrix * observation;

  return Eigen::Vector2f(observed_landmark_absolute_position(0),
                         observed_landmark_absolute_position(1));
}

Eigen::MatrixXf ObservationModel::get_gv(const Eigen::VectorXf &expected_state,
                                         const ObservationData &observation_data) const {
  Eigen::MatrixXf gv(2, 3);  // Initialize a 2x3 matrix

  gv << 1, 0,
      -observation_data.position.x * sin(expected_state(2)) -
          observation_data.position.y * cos(expected_state(2)),
      0, 1,
      observation_data.position.x * cos(expected_state(2)) -
          observation_data.position.y * sin(expected_state(2));

  return gv;
}

Eigen::MatrixXf ObservationModel::get_gz(const Eigen::VectorXf &expected_state,
                                         const ObservationData &observation_data) const {
  Eigen::MatrixXf gz(2, 2);  // Initialize a 2x2 matrix

  gz << cos(expected_state(2)), -sin(expected_state(2)), sin(expected_state(2)),
      cos(expected_state(2));

  return gz;
}

Eigen::Vector2f ObservationModel::observation_model(const Eigen::VectorXf &expected_state,
                                                    const unsigned int landmark_index) const {
  Eigen::Matrix3f transformation_matrix = Eigen::Matrix3f::Identity();
  // transformation matrix is the matrix that transforms the landmark position from the world
  // coordinate system to the robot coordinate system for future comparison with the observed
  // landmark position

  transformation_matrix(0, 0) = cos(-expected_state(2));
  transformation_matrix(0, 1) = -sin(-expected_state(2));
  transformation_matrix(0, 2) =
      -expected_state(0) * cos(-expected_state(2)) + expected_state(1) * sin(-expected_state(2));
  transformation_matrix(1, 0) = sin(-expected_state(2));
  transformation_matrix(1, 1) = cos(-expected_state(2));
  transformation_matrix(1, 2) =
      -expected_state(0) * sin(-expected_state(2)) - expected_state(1) * cos(-expected_state(2));
  Eigen::Vector3f observation =
      transformation_matrix *
      Eigen::Vector3f(static_cast<float>(expected_state(landmark_index)),
                      static_cast<float>(expected_state(landmark_index + 1)), 1);

  return Eigen::Vector2f(observation(0), observation(1));
}

Eigen::VectorXf ObservationModel::format_observation(
    const std::vector<Eigen::Vector2f> &observations) const {
  Eigen::VectorXf formatted_observation(2 * observations.size());
  for (int i = 0; i < static_cast<int>(observations.size()); i++) {
    formatted_observation(2 * i) = observations[i](0);
    formatted_observation(2 * i + 1) = observations[i](1);
  }
  return formatted_observation;
}

Eigen::VectorXf ObservationModel::observation_model_n_landmarks(
    const Eigen::VectorXf &current_state, const std::vector<int> &matched_ids) const {
  auto number_of_landmarks = static_cast<int>(matched_ids.size());
  Eigen::VectorXf expected_observation(2 * number_of_landmarks);
  // transformation matrix is the matrix that transforms the landmark position from the world
  // coordinate system to the robot coordinate system for future comparison with the observed
  // landmark position

  double cossine_minus_theta = cos(-current_state(2));
  double sine_minus_theta = sin(-current_state(2));

  double car_shift_x =
      -current_state(0) * cossine_minus_theta + current_state(1) * sine_minus_theta;
  double car_shift_y =
      -current_state(0) * sine_minus_theta - current_state(1) * cossine_minus_theta;

  unsigned int j = 0;
  for (unsigned int i : matched_ids) {
    expected_observation(2 * j) = cossine_minus_theta * current_state(i) -
                                  sine_minus_theta * current_state(i + 1) + car_shift_x;
    expected_observation(2 * j + 1) = sine_minus_theta * current_state(i) +
                                      cossine_minus_theta * current_state(i + 1) + car_shift_y;
    j++;
  }

  return expected_observation;
}

Eigen::MatrixXf ObservationModel::get_jacobian_of_observation_model(
    const Eigen::VectorXf &current_state, const std::vector<int> &matched_ids) const {
  int number_of_landmarks = matched_ids.size();
  double cossine_minus_theta = cos(-current_state(2));
  double sine_minus_theta = sin(-current_state(2));
  Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(number_of_landmarks * 2, current_state.size());
  for (int i = 0; i < number_of_landmarks; i++) {
    jacobian(2 * i, 0) = -cossine_minus_theta;
    jacobian(2 * i + 1, 0) = -sine_minus_theta;
  }
  for (int i = 0; i < number_of_landmarks; i++) {
    jacobian(2 * i, 1) = sine_minus_theta;
    jacobian(2 * i + 1, 1) = -cossine_minus_theta;
  }
  for (int i = 0; i < number_of_landmarks; i++) {
    jacobian(2 * i, 2) =
        sine_minus_theta * (current_state(matched_ids[i]) - current_state(0)) +
        cossine_minus_theta * (current_state(matched_ids[i] + 1) - current_state(1));
    jacobian(2 * i + 1, 2) =
        -cossine_minus_theta * (current_state(matched_ids[i]) - current_state(0)) +
        sine_minus_theta * (current_state(matched_ids[i] + 1) - current_state(1));
  }
  for (int i = 0; i < number_of_landmarks; i++) {
    jacobian(2 * i, matched_ids[i]) = cossine_minus_theta;
    jacobian(2 * i, matched_ids[i] + 1) = -sine_minus_theta;
    jacobian(2 * i + 1, matched_ids[i]) = sine_minus_theta;
    jacobian(2 * i + 1, matched_ids[i] + 1) = cossine_minus_theta;
  }
  return jacobian;
}

Eigen::MatrixXf ObservationModel::get_state_to_observation_matrix(
    const Eigen::VectorXf &expected_state, const unsigned int landmark_index,
    const unsigned int state_size) const {
  // Old Implementation
  Eigen::MatrixXf reformating_matrix = Eigen::MatrixXf::Zero(8, state_size);
  reformating_matrix(0, 0) = 1;
  reformating_matrix(1, 1) = 1;
  reformating_matrix(2, 2) = 1;
  reformating_matrix(3, 3) = 1;
  reformating_matrix(4, 4) = 1;
  reformating_matrix(5, 5) = 1;
  reformating_matrix(6, landmark_index) = 1;
  reformating_matrix(7, landmark_index + 1) = 1;

  // partial derivative of h(x) or z_hat with respect to
  // (x,y,theta,landmark_x,landmark_y)
  Eigen::MatrixXf low_jacobian = Eigen::MatrixXf::Zero(2, 8);

  // the function h that is derived is the observation model function which after the matrix
  // multiplication becomes:
  //  expected_landmark_x = I * cos(t) + m * sin(t) - x * cos(t) - y * sin(t); -> FUNC_X
  //  expected_landmark_y = -I * sin(t) + m * cos(t) + x * sin(t) - y * cos(t); -> FUNC_Y
  // and so the low_jacobian below is:
  // FUNC_X partially derived in x, y, theta, vx, vy, omega, m, n
  // FUNC_Y partially derived in x, y, theta, vx, vy, omega, m, n
  // where x is the x coordinate of the robot, y is the y coordinate of the robot, theta is the
  // orientation of the robot, m is the x coordinate of the landmark, n is the y coordinate of the
  // landmark
  low_jacobian(0, 0) = -cos(-expected_state(2));
  low_jacobian(0, 1) = sin(-expected_state(2));
  low_jacobian(0, 2) = -expected_state(landmark_index) * sin(expected_state(2)) +
                       expected_state(landmark_index + 1) * cos(-expected_state(2)) +
                       expected_state(0) * sin(expected_state(2)) -
                       expected_state(1) * cos(-expected_state(2));
  low_jacobian(0, 3) = 0;
  low_jacobian(0, 4) = 0;
  low_jacobian(0, 5) = 0;
  low_jacobian(0, 6) = cos(-expected_state(2));
  low_jacobian(0, 7) = -sin(-expected_state(2));

  low_jacobian(1, 0) = -sin(-expected_state(2));
  low_jacobian(1, 1) = -cos(-expected_state(2));
  low_jacobian(1, 2) = -expected_state(landmark_index) * cos(-expected_state(2)) -
                       expected_state(landmark_index + 1) * sin(expected_state(2)) +
                       expected_state(0) * cos(-expected_state(2)) +
                       expected_state(1) * sin(expected_state(2));
  low_jacobian(1, 3) = 0;
  low_jacobian(1, 4) = 0;
  low_jacobian(1, 5) = 0;
  low_jacobian(1, 6) = sin(-expected_state(2));
  low_jacobian(1, 7) = cos(-expected_state(2));

  Eigen::MatrixXf validation_jacobian = low_jacobian * reformating_matrix;

  return validation_jacobian;
}

Eigen::MatrixXf ObservationModel::get_observation_noise_covariance_matrix() const {
  return this->_observation_noise_covariance_matrix_;
}

Eigen::MatrixXf ObservationModel::get_full_observation_noise_covariance_matrix(
    const int observation_size) const {
  double noise_value = this->get_observation_noise_covariance_matrix()(0, 0);
  return Eigen::MatrixXf::Identity(observation_size, observation_size) * noise_value;
}

Eigen::MatrixXf ObservationModel::create_observation_noise_covariance_matrix(float noise_value) {
  return Eigen::MatrixXf::Identity(2, 2) * noise_value;
}