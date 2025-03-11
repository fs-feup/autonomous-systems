#include "perception_sensor_lib/observation_model/base_observation_model.hpp"

#include "common_lib/maths/transformations.hpp"

Eigen::VectorXd ObservationModel::observation_model(
    const Eigen::VectorXd& state, const std::vector<int> matched_landmarks) const {
  Eigen::VectorXd matched_landmarks_coordinates(matched_landmarks.size() * 2);
  for (int i = 0; i < static_cast<int>(matched_landmarks.size()); i++) {
    matched_landmarks_coordinates(2 * i) = state(matched_landmarks[i]);
    matched_landmarks_coordinates(2 * i + 1) = state(matched_landmarks[i] + 1);
  }
  return common_lib::maths::global_to_local_coordinates(state.segment(0, 3),
                                                        matched_landmarks_coordinates);
}

Eigen::VectorXd ObservationModel::inverse_observation_model(
    const Eigen::VectorXd& state, const Eigen::VectorXd& observations) const {
  return common_lib::maths::local_to_global_coordinates(state.segment(0, 3), observations);
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