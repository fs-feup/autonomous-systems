
#include <Eigen/Dense>
#include <iostream>

#include "kalman_filter/ekf.hpp"
#include "loc_map/data_structures.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(Eigen::MatrixXf R, Eigen::MatrixXf Q,
                                           VehicleState* vehicle_state, Map* map,
                                           ImuUpdate* imu_update, Map* map_from_perception,
                                           const MotionModel& motion_model)
    : X(Eigen::VectorXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200, 200)),
      R(R),
      Q(Q),
      _vehicle_state(vehicle_state),
      _map(map),
      _imu_update(imu_update),
      _map_from_perception(map_from_perception),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model) {}

void ExtendedKalmanFilter::prediction_step() {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  MotionPredictionData prediction_data = {
      this->_imu_update->translational_velocity, this->_imu_update->translational_velocity_x,
      this->_imu_update->translational_velocity_y, this->_imu_update->rotational_velocity};
  X = this->_motion_model.motion_model_expected_state(X, prediction_data, delta / 1000000);
  P = this->_motion_model.motion_model_covariance_matrix(P, this->R, prediction_data,
                                                         delta / 1000000);
  this->_last_update = now;
}

void ExtendedKalmanFilter::correction_step() {
  // TODO(marhcouto): implement this
  // for (auto cone : this->_map_from_perception->map) {
  //   double absolute_x = cone.first.x + this->_vehicle_state->pose.position.x;
  //   double absolute_y = cone.first.y + this->_vehicle_state->pose.position.y;

  // }
}

void ExtendedKalmanFilter::update() { this->_vehicle_state->pose = Pose(X(0), X(1), X(2)); }
