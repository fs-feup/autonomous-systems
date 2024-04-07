#define EIGEN_USE_THREADS

#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>

#include "speed_est/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"

double ExtendedKalmanFilter::max_landmark_distance = 71.0;

bool ExtendedKalmanFilter::cone_match(const double x_from_state, const double y_from_state,
                                      const double x_from_perception,
                                      const double y_from_perception,
                                      const double distance_to_vehicle) {
  double delta_x = x_from_state - x_from_perception;
  double delta_y = y_from_state - y_from_perception;
  double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
  auto limit_function = [](double distance) {
    double curvature = 8.0;
    double initial_limit = 0.5;
    return pow(M_E, distance / curvature) - (1 - initial_limit);
  };
  return limit_function(distance_to_vehicle) > delta;
}
/*---------------------- Tests helper functions --------------------*/
/**
 * @brief set value of X at y to parameter value
 *
 */
void ExtendedKalmanFilter::set_X_y(int y, float value) { this->X(y) = value; }
void ExtendedKalmanFilter::push_to_colors(colors::Color color) { _colors.push_back(color); }

void ExtendedKalmanFilter::set_P(int size) {
  this->P = Eigen::SparseMatrix<float>(size, size);
  this->P.coeffRef(0, 0) = 1.1;
  this->P.coeffRef(1, 1) = 1.1;
  this->P.coeffRef(2, 2) = 1.1;
}

/**
 * @brief Initializes state X with size equal to parameter size
 */
void ExtendedKalmanFilter::init_X_size(int size) { this->X = Eigen::VectorXf::Zero(size); }

/*---------------------- Constructors --------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(const MotionModel &motion_model,
                                           const ObservationModel &observation_model)
    : X(Eigen::VectorXf::Zero(3)),
      P(3, 3),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model),
      _observation_model(observation_model),
      _fixed_map(false) {
  P.setZero();
}
/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::prediction_step(const MotionUpdate &motion_update) {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  this->_last_motion_update = motion_update;
  Eigen::VectorXf tempX = this->X;
  this->X = this->_motion_model.predict_expected_state(tempX, motion_update, delta / 1000000);
  Eigen::MatrixXf G =
      this->_motion_model.get_motion_to_state_matrix(tempX, motion_update, delta / 1000000);
  Eigen::MatrixXf R = this->_motion_model.get_process_noise_covariance_matrix(
      this->X.size());  // Process Noise Matrix
  // this->P = G * this->P * G.transpose() + R;
  Eigen::MatrixXf P_dense = G * Eigen::MatrixXf(this->P) * G.transpose() + R;
  this->P = P_dense.sparseView();
  this->_last_update = now;
}

void ExtendedKalmanFilter::correction_step(const ConeMap &perception_map) {
  for (auto cone : perception_map.map) {
    ObservationData observation_data = ObservationData(cone.first.x, cone.first.y, cone.second);
    int landmark_index = this->discovery(observation_data);
    if (landmark_index == -1) {  // Too far away landmark
      continue;
    }
    Eigen::MatrixXf H = this->_observation_model.get_state_to_observation_matrix(
        this->X, landmark_index, this->X.size());
    Eigen::MatrixXf Q =
        this->_observation_model.get_observation_noise_covariance_matrix();  // Observation Noise
                                                                             // Matrix

    Eigen::MatrixXf K = this->get_kalman_gain(H, this->P, Q);
    Eigen::Vector2f z_hat = this->_observation_model.observation_model(
        this->X, landmark_index);  // expected observation, previously calculated landmark is used
    Eigen::Vector2f z =
        Eigen::Vector2f(observation_data.position.x,
                        observation_data.position.y);  // position of the landmark  observed
    this->X = this->X + K * (z - z_hat);
    // TODO(PedroRomao3): divide into multiple calculation and measure time.
    // TODO(PedroRomao3): Diagram.
    this->P =
        (Eigen::MatrixXf::Identity(this->P.rows(), this->P.cols()) - K * H).sparseView() * this->P;
  }
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf &H,
                                                      const Eigen::MatrixXf &P,
                                                      const Eigen::MatrixXf &Q) {
  Eigen::MatrixXf S = H * P * H.transpose() + Q;
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  return K;
}

int ExtendedKalmanFilter::discovery(const ObservationData &observation_data) {
  Eigen::Vector2f landmark_absolute =
      this->_observation_model.inverse_observation_model(this->X, observation_data);
  double distance =
      std::sqrt(pow(observation_data.position.x, 2) + pow(observation_data.position.y, 2));
  if (distance > ExtendedKalmanFilter::max_landmark_distance) {
    return -1;
  }
  double best_delta = 1000000000.0;
  int best_index = -1;
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    double delta_x = this->X(i) - landmark_absolute(0);
    double delta_y = this->X(i + 1) - landmark_absolute(1);
    double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
    if (delta < best_delta && this->_colors[(i - 3) / 2] == observation_data.color) {
      best_index = i;
      best_delta = delta;
    }
  }
  if (best_index != -1) {
    double score =
        ExtendedKalmanFilter::cone_match(this->X(best_index), this->X(best_index + 1),
                                         landmark_absolute(0), landmark_absolute(1), distance);
    if (score > 0) {
      return best_index;
    }
  }

  // If not found, add to the map
  this->X.conservativeResizeLike(Eigen::VectorXf::Zero(this->X.size() + 2));
  this->X(this->X.size() - 2) = landmark_absolute(0);
  this->X(this->X.size() - 1) = landmark_absolute(1);
  // this->P.conservativeResizeLike(Eigen::MatrixXf::Zero(this->P.rows() + 2, this->P.cols() + 2));

  // Create a new sparse matrix of the correct size
  Eigen::SparseMatrix<float> newP(this-> P.rows() + 2, this->P.cols() + 2);

  // Copy the values from P into newP
  for (int i = 0; i < this->P.rows(); i++) {
    for (int j = 0; j < this->P.cols(); j++) {
      newP.coeffRef(i, j) = this->P.coeff(i, j);
    }
  }

  // Replace P with newP
  this->P.swap(newP);

  this->_colors.push_back(observation_data.color);
  return this->X.size() - 2;
}

void ExtendedKalmanFilter::update(VehicleState *vehicle_state, ConeMap *track_map) {
  vehicle_state->pose = Pose(X(0), X(1), X(2));
  track_map->map.clear();
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    track_map->map.insert(
        std::pair<Position, colors::Color>(Position(X(i), X(i + 1)), this->_colors[(i - 3) / 2]));
  }
}
