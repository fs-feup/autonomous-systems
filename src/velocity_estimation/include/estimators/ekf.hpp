#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/structures/velocities.hpp"
#include "estimators/estimator.hpp"

class EKF : public VelocityEstimator {
  Eigen::Vector3f state_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance_ = Eigen::Matrix3f::Identity();
  common_lib::sensor_data::ImuData imu_data_;
  common_lib::sensor_data::WheelEncoderData wss_data_;
  double resolver_data_;
  double steering_data_;
  void predict(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               common_lib::sensor_data::ImuData& imu_data);
  void correct(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               common_lib::sensor_data::WheelEncoderData& wss_data, double resolver_data,
               double steering_data);

public:
  EKF() = default;
  void IMUCallback(const common_lib::sensor_data::ImuData& imu_data) override;
  void WSSCallback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void ResolverCallback(double resolver_data) override;
  void SteeringCallback(double steering_data) override;
  common_lib::structures::Velocities get_velocities() override;
};