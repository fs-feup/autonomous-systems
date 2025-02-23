#include "perception_sensor_lib/observation_model/base_observation_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::VectorXd ObservationModel::observation_model(
    const Eigen::VectorXd& state, const std::vector<int> matched_landmarks) const {
  double car_x = state(0);
  double car_y = state(1);
  double car_orientation = state(2);

  Eigen::Matrix2d rotation_matrix = common_lib::maths::get_rotation_matrix(-car_orientation);

  int num_landmarks = static_cast<int>(matched_landmarks.size());
  Eigen::VectorXd observations = Eigen::VectorXd(num_landmarks * 2);

  for (int i = 0; i < num_landmarks; ++i) {
    Eigen::Vector2d landmark_global(state(matched_landmarks[i]), state(matched_landmarks[i] + 1));
    Eigen::Vector2d landmark_relative =
        rotation_matrix * (landmark_global - Eigen::Vector2d(car_x, car_y));
    observations(2 * i) = landmark_relative(0);
    observations(2 * i + 1) = landmark_relative(1);
  }
  return observations;
}

Eigen::VectorXd ObservationModel::inverse_observation_model(
    const Eigen::VectorXd& state, const Eigen::VectorXd& observations) const {
  double car_x = state(0);
  double car_y = state(1);
  double car_orientation = state(2);

  Eigen::Matrix2d rotation_matrix = common_lib::maths::get_rotation_matrix(car_orientation);

  int num_landmarks = observations.size() / 2;

  Eigen::VectorXd landmarks_global(num_landmarks * 2);

  for (int i = 0; i < num_landmarks; ++i) {
    Eigen::Vector2d landmark_relative(observations(2 * i), observations(2 * i + 1));

    Eigen::Vector2d landmark_global =
        rotation_matrix * landmark_relative + Eigen::Vector2d(car_x, car_y);

    landmarks_global(2 * i) = landmark_global(0);
    landmarks_global(2 * i + 1) = landmark_global(1);
  }

  return landmarks_global;
}

Eigen::MatrixXd ObservationModel::observation_model_jacobian(
    const Eigen::VectorXd& state, const std::vector<int>& matched_landmarks) const {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(matched_landmarks.size() * 2, state.size());
  double car_x = state(0);
  double car_y = state(1);
  double car_orientation = state(2);

  int num_landmarks = static_cast<int>(matched_landmarks.size());

  for (int i = 0; i < num_landmarks; ++i) {
    jacobian(i, 0) = -cos(car_orientation);
    jacobian(i + 1, 0) = sin(car_orientation);
    jacobian(i, 1) = -sin(car_orientation);
    jacobian(i + 1, 1) = -cos(car_orientation);
    jacobian(i, 2) = -sin(car_orientation) * (state(matched_landmarks[i]) - car_x) +
                     cos(car_orientation) * (state(matched_landmarks[i] + 1) - car_y);
    jacobian(i + 1, 2) = -cos(car_orientation) * (state(matched_landmarks[i]) - car_x) -
                         sin(car_orientation) * (state(matched_landmarks[i] + 1) - car_y);
    jacobian(i, matched_landmarks[i]) = cos(car_orientation);
    jacobian(i + 1, matched_landmarks[i]) = -sin(car_orientation);
    jacobian(i, matched_landmarks[i] + 1) = sin(car_orientation);
    jacobian(i + 1, matched_landmarks[i] + 1) = cos(car_orientation);
  }
  return jacobian;
}

Eigen::MatrixXd ObservationModel::inverse_observation_model_jacobian_pose(
    const Eigen::VectorXd& state, const Eigen::VectorXd& new_landmarks) const {
  int num_landmarks = new_landmarks.size() / 2;
  Eigen::MatrixXd gv = Eigen::MatrixXd::Zero(num_landmarks * 2, 3);
  for (int i = 0; i < num_landmarks; i++) {
    gv(2 * i, 0) = 1;
    gv(2 * i, 1) = 0;
    gv(2 * i, 2) = -new_landmarks(2 * i) * sin(state(2)) - new_landmarks(2 * i + 1) * cos(state(2));
    gv(2 * i + 1, 0) = 0;
    gv(2 * i + 1, 1) = 1;
    gv(2 * i + 1, 2) =
        new_landmarks(2 * i) * cos(state(2)) - new_landmarks(2 * i + 1) * sin(state(2));
  }
  return gv;
}

Eigen::MatrixXd ObservationModel::inverse_observation_model_jacobian_landmarks(
    const Eigen::VectorXd& state, const Eigen::VectorXd& new_landmarks) const {
  int num_landmarks = new_landmarks.size() / 2;
  Eigen::MatrixXd gz = Eigen::MatrixXd::Zero(num_landmarks * 2, num_landmarks * 2);
  for (int i = 0; i < num_landmarks; i++) {
    gz(2 * i, 2 * i) = cos(state(2));
    gz(2 * i, 2 * i + 1) = -sin(state(2));
    gz(2 * i + 1, 2 * i) = sin(state(2));
    gz(2 * i + 1, 2 * i + 1) = cos(state(2));
  }
  return gz;
}