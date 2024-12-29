#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>

#include "adapters/parameters.hpp"
#include "common_lib/structures/velocities.hpp"
#include "estimators/estimator.hpp"
#include "motion_lib/particle_model.hpp"

class EKF : public VelocityEstimator {
  std::chrono::high_resolution_clock::time_point last_update_;
  Eigen::Vector3f state_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f process_noise_matrix_;

  common_lib::sensor_data::ImuData imu_data_;
  common_lib::sensor_data::WheelEncoderData wss_data_;
  double resolver_data_;
  double steering_data_;

  // The following flags are used to keep track of which sensors have provided at least one
  // measurement. This is necessary to ensure that the estimator is not used before all sensors have
  // provided at least one measurement.
  bool imu_data_received_ = false;
  bool wss_data_received_ = false;
  bool resolver_data_received_ = false;
  bool steering_data_received_ = false;

  /**
   * @brief Predict velocities at the next index based on IMU measurements and current state
   *
   * @param state Vector of velocities {velocity_x, velocity_y, rotational_velocity}
   * @param covariance Covariance matrix representing the uncertainty in the state estimate.
   * @param process_noise_matrix Process noise matrix representing the uncertainty in the process
   * model.
   * @param last_update Time point of the last update.
   * @param imu_data IMU data containing acceleration and rotational velocity measurements.
   */
  void predict(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               const Eigen::Matrix3f& process_noise_matrix,
               const std::chrono::high_resolution_clock::time_point last_update,
               common_lib::sensor_data::ImuData& imu_data);

  /**
   * @brief Correct the state estimate based on wheel speed sensor, resolver, and steering data.
   *
   * This function updates the state and covariance estimates using measurements from the wheel
   * speed sensor, resolver, and steering angle sensor. It corrects the predicted state to better
   * match the observed measurements.
   *
   * @param state Vector of velocities {velocity_x, velocity_y, rotational_velocity}
   * @param covariance Covariance matrix representing the uncertainty in the state estimate.
   * @param wss_data Wheel speed sensor data containing wheel speeds.
   * @param resolver_data Resolver data representing the motor's rpms.
   * @param steering_data Steering data representing the steering angle.
   */
  void correct(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               common_lib::sensor_data::WheelEncoderData& wss_data, double resolver_data,
               double steering_data);

public:
  EKF(const VEParameters& params);
  void IMUCallback(const common_lib::sensor_data::ImuData& imu_data) override;
  void WSSCallback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void ResolverCallback(double resolver_data) override;
  void SteeringCallback(double steering_data) override;
  common_lib::structures::Velocities get_velocities() override;
};