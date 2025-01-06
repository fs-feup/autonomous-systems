#pragma once

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>

#include "adapters/parameters.hpp"
#include "common_lib/structures/velocities.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/estimator.hpp"
#include "motion_lib/particle_model.hpp"

class EKF : public VelocityEstimator {
  std::chrono::high_resolution_clock::time_point last_update_;
  Eigen::Vector3f state_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f process_noise_matrix_;
  Eigen::MatrixXf measurement_noise_matrix_;

  common_lib::sensor_data::ImuData imu_data_;
  common_lib::sensor_data::WheelEncoderData wss_data_;
  double motor_rpm_;
  double steering_angle_;

  // The following flags are used to keep track of which sensors have provided at least one
  // measurement. This is necessary to ensure that the estimator is not used before all sensors have
  // provided at least one measurement.
  bool imu_data_received_ = false;
  bool wss_data_received_ = false;
  bool motor_rpm_received_ = false;
  bool steering_angle_received_ = false;

  // Parameters
  double wheel_base_;
  double weight_distribution_front_;
  double wheel_radius_;
  double gear_ratio_;

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
   * @param motor_rpm data representing the motor's rpms.
   * @param steering_angle data representing the steering angle.
   */
  void correct(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
               common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
               double steering_angle);

  /**
   * @brief Estimate observations assuming a bicycle model and a set of parameters
   *
   * @param state vector of velocities {velocity_x, velocity_y, rotational_velocity}
   * @param wheel_base distance between the front and rear wheels
   * @param weight_distribution_front percentage of the vehicle's weight on the front wheels
   * @param gear_ratio rations of the motor for each rotation of the rear wheels
   * @param wheel_radius radius of the rear wheels
   * @return Eigen::VectorXf vector of observations {fl_rpm, fr_rpm, rl_rpm, rr_rpm, steering_angle,
   * motor_rpm}
   */
  Eigen::VectorXf estimate_observations(Eigen::Vector3f& state, double wheel_base,
                                        double weight_distribution_front, double gear_ratio,
                                        double wheel_radius);

  Eigen::MatrixXf jacobian_of_observation_estimation(Eigen::Vector3f& state, double wheel_base,
                                                     double weight_distribution_front,
                                                     double gear_ratio, double wheel_radius);

public:
  EKF(const VEParameters& params);
  void imu_callback(const common_lib::sensor_data::ImuData& imu_data) override;
  void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void motor_rpm_callback(double motor_rpm) override;
  void steering_callback(double steering_angle) override;
  common_lib::structures::Velocities get_velocities() override;
};