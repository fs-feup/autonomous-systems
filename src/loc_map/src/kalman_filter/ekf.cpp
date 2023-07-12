
#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <iostream>

#include "loc_map/data_structures.hpp"

float ExtendedKalmanFilter::max_landmark_deviation = 0.1;

ExtendedKalmanFilter::ExtendedKalmanFilter(VehicleState* vehicle_state, Map* map,
                                           MotionUpdate* imu_update, Map* map_from_perception,
                                           const MotionModel& motion_model,
                                           const ObservationModel& observation_model)
    : X(Eigen::VectorXf::Zero(3)),
      P(Eigen::MatrixXf::Zero(3, 3)),
      _vehicle_state(vehicle_state),
      _map(map),
      _motion_update(imu_update),
      _map_from_perception(map_from_perception),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model),
      _observation_model(observation_model) {}

void ExtendedKalmanFilter::prediction_step() {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  MotionPredictionData prediction_data = {
      this->_motion_update->translational_velocity, this->_motion_update->translational_velocity_x,
      this->_motion_update->translational_velocity_y, this->_motion_update->rotational_velocity,
      this->_motion_update->steering_angle};
  this->X = this->_motion_model.predict_expected_state(X, prediction_data, delta / 1000000);
  Eigen::MatrixXf G =
      this->_motion_model.get_motion_to_state_matrix(X, prediction_data, delta / 1000000);
  Eigen::MatrixXf R = this->_motion_model.get_process_noise_covariance_matrix(
      this->X.size());  // Process Noise Matrix
  this->P = G * this->P * G.transpose() + R;
  this->_last_update = now;
}

void ExtendedKalmanFilter::correction_step() {
  for (auto cone : this->_map_from_perception->map) {
    ObservationData observation_data = ObservationData(cone.first.x, cone.first.y, cone.second);
    unsigned int landmark_index = this->discovery(observation_data);
    Eigen::MatrixXf H = this->_observation_model.get_state_to_observation_matrix(
        this->X, landmark_index, this->X.size());
    Eigen::MatrixXf Q = this->_observation_model
                            .get_observation_noise_covariance_matrix();  // Observation Noise Matrix
    Eigen::MatrixXf K = this->get_kalman_gain(H, this->P, Q);
    Eigen::Vector2f z_hat = this->_observation_model.observation_model(this->X, landmark_index);
    Eigen::Vector2f z = Eigen::Vector2f(observation_data.position.x, observation_data.position.y);
    this->X = this->X + K * (z - z_hat);
    this->P = (Eigen::MatrixXf::Identity(this->P.rows(), this->P.cols()) - K * H) * this->P;
  }
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf& H,
                                                      const Eigen::MatrixXf& P,
                                                      const Eigen::MatrixXf& Q) {
  Eigen::MatrixXf S = H * P * H.transpose() + Q;  // TODO(marhcouto): Add Q
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  return K;
}

unsigned int ExtendedKalmanFilter::discovery(const ObservationData& observation_data) {
  Eigen::Vector2f landmark_absolute =
      this->_observation_model.inverse_observation_model(this->X, observation_data);
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    double delta_x = this->X(i) - landmark_absolute(0);
    double delta_y = this->X(i + 1) - landmark_absolute(1);
    double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
    if (delta <= ExtendedKalmanFilter::max_landmark_deviation &&
        this->_colors[(i - 3) / 2] == observation_data.color)
      return i;
  }
  // If not found, add to the map
  this->X.conservativeResizeLike(Eigen::VectorXf::Zero(this->X.size() + 2));
  this->X(this->X.size() - 2) = landmark_absolute(0);
  this->X(this->X.size() - 1) = landmark_absolute(1);
  this->P.conservativeResizeLike(Eigen::MatrixXf::Zero(this->P.rows() + 2, this->P.cols() + 2));
  this->_colors.push_back(observation_data.color);
  return this->X.size() - 2;
}

void ExtendedKalmanFilter::update() {
  this->_vehicle_state->pose = Pose(X(0), X(1), X(2));
  this->_map->map.clear();
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    this->_map->map.insert(
        std::pair<Position, colors::Color>(Position(X(i), X(i + 1)), this->_colors[(i - 3) / 2]));
  }
}
