#pragma once

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <memory>

#include "common_lib/structures/velocities.hpp"
#include "config/parameters.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/estimator.hpp"
#include "motion_lib/vel_process_model/map.hpp"
#include "perception_sensor_lib/observation_model/ve_observation_model/map.hpp"


class NoRearWSSEKF : public VelocityEstimator {
  rclcpp::Time _last_update_;
  Eigen::Vector3d _state_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d _covariance_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d _process_noise_matrix_;
  Eigen::MatrixXd _wheels_measurement_noise_matrix_;
  Eigen::MatrixXd _imu_measurement_noise_matrix_;

  common_lib::sensor_data::ImuData imu_data_;
  common_lib::sensor_data::WheelEncoderData wss_data_;
  double motor_rpm_;
  double steering_angle_;

  bool _has_made_prediction_ = false;

  // The following flags are used to keep track of which sensors have provided at least one
  // measurement. This is necessary to ensure that the estimator is not used before all sensors have
  // provided at least one measurement.
  bool imu_data_received_ = false;
  bool wss_data_received_ = false;
  bool motor_rpm_received_ = false;
  bool steering_angle_received_ = false;

  common_lib::car_parameters::CarParameters car_parameters_;
  std::shared_ptr<VEObservationModel> observation_model_;
  std::shared_ptr<BaseVelocityProcessModel> process_model;

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
  void predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
               const Eigen::Matrix3d& process_noise_matrix, const rclcpp::Time last_update,
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
   * @param motor_rpm data representing the motor's rpms.
   * @param steering_angle data representing the steering angle.
   */
  void correct_wheels(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                      common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
                      double steering_angle);

  void correct_imu(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                   common_lib::sensor_data::ImuData& imu_data);

public:
  NoRearWSSEKF(const VEParameters& params);
  /**
   * @brief Callback function for the IMU data that should be called by adapters when new IMU data
   * is available.
   */
  void imu_callback(const common_lib::sensor_data::ImuData& imu_data) override;
  /**
   * @brief Callback function for the wheel speed sensor data that should be called by adapters when
   * new wheel speed sensor data is available.
   */
  void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  /**
   * @brief Callback function for the motor RPM data that should be called by adapters when new
   * motor RPM data is available.
   */
  void motor_rpm_callback(double motor_rpm) override;
  /**
   * @brief Callback function for the steering angle data that should be called by adapters when new
   * steering angle data is available.
   */
  void steering_callback(double steering_angle) override;
  common_lib::structures::Velocities get_velocities() override;
};