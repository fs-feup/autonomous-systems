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

  bool imu_data_received_ = false;
  bool wss_data_received_ = false;
  bool resolver_data_received_ = false;
  bool steering_data_received_ = false;

  void predict(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               const Eigen::Matrix3f& process_noise_matrix,
               const std::chrono::high_resolution_clock::time_point last_update,
               common_lib::sensor_data::ImuData& imu_data);

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