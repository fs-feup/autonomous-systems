
#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <iostream>

#include "loc_map/data_structures.hpp"

double ExtendedKalmanFilter::max_landmark_distance = 5.0;

bool ExtendedKalmanFilter::cone_match(const double x_from_state, const double y_from_state,
                                      const double x_from_perception,
                                      const double y_from_perception,
                                      const double distance_to_vehicle) {
  double delta_x = x_from_state - x_from_perception;
  double delta_y = y_from_state - y_from_perception;
  double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
  auto limit_function = [](double distance) {
    double curvature = 12.0;
    double initial_limit = 0.5;
    return pow(M_E, distance / curvature) - (1 - initial_limit);
  };
  return delta <= limit_function(distance_to_vehicle);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const MotionModel& motion_model,
                                           const ObservationModel& observation_model)
    : X(Eigen::VectorXf::Zero(3)),
      P(Eigen::MatrixXf::Zero(3, 3)),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model),
      _observation_model(observation_model) {}

void ExtendedKalmanFilter::prediction_step(const MotionUpdate& motion_update) {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  this->_last_motion_update = motion_update;
  this->X = this->_motion_model.predict_expected_state(X, motion_update, delta / 1000000);
  Eigen::MatrixXf G =
      this->_motion_model.get_motion_to_state_matrix(X, motion_update, delta / 1000000);
  Eigen::MatrixXf R = this->_motion_model.get_process_noise_covariance_matrix(
      this->X.size());  // Process Noise Matrix
  this->P = G * this->P * G.transpose() + R;
  this->_last_update = now;
}

void ExtendedKalmanFilter::correction_step(const Map& perception_map) {
  for (auto cone : perception_map.map) {
    ObservationData observation_data = ObservationData(cone.first.x, cone.first.y, cone.second);
    int landmark_index = this->discovery(observation_data);
    if (landmark_index == -1) {  // Too far away landmark
      continue;
    }
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
  Eigen::MatrixXf S = H * P * H.transpose() + Q;
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  return K;
}

int ExtendedKalmanFilter::discovery(const ObservationData& observation_data) {
  Eigen::Vector2f landmark_absolute =
      this->_observation_model.inverse_observation_model(this->X, observation_data);
  double distance =
      std::sqrt(pow(observation_data.position.x, 2) + pow(observation_data.position.y, 2));
  if (distance > ExtendedKalmanFilter::max_landmark_distance) {
    return -1;
  }
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    if (ExtendedKalmanFilter::cone_match(this->X(i), this->X(i + 1), observation_data.position.x,
                                         observation_data.position.y, distance) &&
        this->_colors[(i - 3) / 2] == observation_data.color) {
      return i;
    }
  }
  // If not found, add to the map
  this->X.conservativeResizeLike(Eigen::VectorXf::Zero(this->X.size() + 2));
  this->X(this->X.size() - 2) = landmark_absolute(0);
  this->X(this->X.size() - 1) = landmark_absolute(1);
  this->P.conservativeResizeLike(Eigen::MatrixXf::Zero(this->P.rows() + 2, this->P.cols() + 2));
  this->_colors.push_back(observation_data.color);
  return this->X.size() - 2;
}

void ExtendedKalmanFilter::update(VehicleState* vehicle_state, Map* track_map) {
  vehicle_state->pose = Pose(X(0), X(1), X(2));
  track_map->map.clear();
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    track_map->map.insert(
        std::pair<Position, colors::Color>(Position(X(i), X(i + 1)), this->_colors[(i - 3) / 2]));
  }
}
