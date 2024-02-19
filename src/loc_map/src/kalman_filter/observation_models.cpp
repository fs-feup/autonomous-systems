#include "kalman_filter/observation_models.hpp"

ObservationModel::ObservationModel(const Eigen::Matrix2f &observation_noise_covariance_matrix)
    : _observation_noise_covariance_matrix(observation_noise_covariance_matrix) {}

Eigen::Vector2f ObservationModel::inverse_observation_model(
    const Eigen::VectorXf &expected_state, const ObservationData &observation_data) const {
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

Eigen::Vector2f ObservationModel::observation_model(const Eigen::VectorXf &expected_state,
                                                    const unsigned int landmark_index) const {
  Eigen::Matrix3f transformation_matrix = Eigen::Matrix3f::Identity();
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
      Eigen::Vector3f(expected_state(landmark_index), expected_state(landmark_index + 1), 1);

  return Eigen::Vector2f(observation(0), observation(1));
}

Eigen::MatrixXf ObservationModel::get_state_to_observation_matrix(
    const Eigen::VectorXf &expected_state, const unsigned int landmark_index,
    const unsigned int state_size) const {
  // Eigen::MatrixXf validation_jacobian = Eigen::MatrixXf::Zero(2, state_size);

  // validation_jacobian(0, 0) = -cos(-expected_state(2));
  // validation_jacobian(1, 0) = -sin(-expected_state(2));
  // validation_jacobian(0, 1) = sin(-expected_state(2));
  // validation_jacobian(1, 1) = -cos(-expected_state(2));
  // validation_jacobian(0, 2) = -expected_state(landmark_index) * sin(expected_state(2)) +
  //                             expected_state(landmark_index + 1) * cos(-expected_state(2)) +
  //                             expected_state(0) * sin(expected_state(2)) -
  //                             expected_state(1) * cos(-expected_state(2));
  // validation_jacobian(1, 2) = -expected_state(landmark_index) * cos(-expected_state(2)) -
  //                             expected_state(landmark_index + 1) * sin(expected_state(2)) +
  //                             expected_state(0) * cos(-expected_state(2)) +
  //                             expected_state(1) * sin(expected_state(2));
  // validation_jacobian(0, landmark_index) = cos(-expected_state(2));
  // validation_jacobian(1, landmark_index) = sin(-expected_state(2));
  // validation_jacobian(0, landmark_index + 1) = -sin(-expected_state(2));
  // validation_jacobian(1, landmark_index + 1) = cos(-expected_state(2));

  // //test with blocks
  // Eigen::MatrixXf block_1 = Eigen::MatrixXf::Identity(3, 3);
  // Eigen::MatrixXf block_2 = Eigen::MatrixXf::Identity(2, 2);

  // //test with sub low_jacobian
  // Eigen::MatrixXf low_jacobian_1 = Eigen::MatrixXf::Zero(2, 3);
  // Eigen::MatrixXf low_jacobian_2 = Eigen::MatrixXf::Zero(2, 2);

  // low_jacobian_1(0, 0) = -cos(-expected_state(2));
  // low_jacobian_1(1, 0) = -sin(-expected_state(2));
  // low_jacobian_1(0, 1) = sin(-expected_state(2));
  // low_jacobian_1(1, 1) = -cos(-expected_state(2));
  // low_jacobian_1(0, 2) = -expected_state(landmark_index) * sin(expected_state(2)) +
  //                      expected_state(landmark_index + 1) * cos(-expected_state(2)) +
  //                      expected_state(0) * sin(expected_state(2)) -
  //                      expected_state(1) * cos(-expected_state(2));
  // low_jacobian_1(1, 2) = -expected_state(landmark_index) * cos(-expected_state(2)) -
  //                      expected_state(landmark_index + 1) * sin(expected_state(2)) +
  //                      expected_state(0) * cos(-expected_state(2)) +
  //                      expected_state(1) * sin(expected_state(2));

  // low_jacobian_2(0, 0) = cos(-expected_state(2));
  // low_jacobian_2(1, 0) = sin(-expected_state(2));
  // low_jacobian_2(0, 1) = -sin(-expected_state(2));
  // low_jacobian_2(1, 1) = cos(-expected_state(2));

  // Eigen::MatrixXf validation_jacobian_1 = low_jacobian_1 * block_1;
  // Eigen::MatrixXf validation_jacobian_2 = low_jacobian_2 * block_2;

  // Eigen::MatrixXf validation_jacobian = Eigen::MatrixXf::Zero(2, state_size);

  // validation_jacobian.block(0, 0, 2, 3) = validation_jacobian_1;
  // validation_jacobian.block(0, landmark_index, 2, 2) = validation_jacobian_2;

  
    Eigen::MatrixXf reformating_matrix = Eigen::MatrixXf::Zero(5, state_size);
    reformating_matrix(0, 0) = 1;
    reformating_matrix(1, 1) = 1;
    reformating_matrix(2, 2) = 1;
    reformating_matrix(3, landmark_index) = 1;
    reformating_matrix(4, landmark_index + 1) = 1;

    Eigen::MatrixXf low_jacobian = Eigen::MatrixXf::Zero(2, 5);
    // partial derivative of h(x) or z_hat (from observation model) with respect to
    // (x,y,theta,landmark_x,landmark_y)
    low_jacobian(0, 0) = -cos(-expected_state(2));
    low_jacobian(1, 0) = -sin(-expected_state(2));
    low_jacobian(0, 1) = sin(-expected_state(2));
    low_jacobian(1, 1) = -cos(-expected_state(2));
    low_jacobian(0, 2) = -expected_state(landmark_index) * sin(expected_state(2)) +
                         expected_state(landmark_index + 1) * cos(-expected_state(2)) +
                         expected_state(0) * sin(expected_state(2)) -
                         expected_state(1) * cos(-expected_state(2));
    low_jacobian(1, 2) = -expected_state(landmark_index) * cos(-expected_state(2)) -
                         expected_state(landmark_index + 1) * sin(expected_state(2)) +
                         expected_state(0) * cos(-expected_state(2)) +
                         expected_state(1) * sin(expected_state(2));
    low_jacobian(0, 3) = cos(-expected_state(2));
    low_jacobian(1, 3) = sin(-expected_state(2));
    low_jacobian(0, 4) = -sin(-expected_state(2));
    low_jacobian(1, 4) = cos(-expected_state(2));

    Eigen::MatrixXf validation_jacobian = low_jacobian * reformating_matrix;

    
  return validation_jacobian;
}

Eigen::MatrixXf ObservationModel::get_observation_noise_covariance_matrix() const {
  return this->_observation_noise_covariance_matrix;
}