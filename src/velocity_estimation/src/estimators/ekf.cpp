#include "estimators/ekf.hpp"

#include <iostream>

EKF::EKF(const VEParameters& params) {
  this->process_noise_matrix_ = Eigen::Matrix3f::Identity() * params._ekf_process_noise_;
  this->measurement_noise_matrix_ =
      Eigen::MatrixXf::Identity(6, 6) * params._ekf_measurement_noise_;
  this->wheel_base_ = params._wheel_base_;
  this->weight_distribution_front_ = params._weight_distribution_front_;
  this->wheel_radius_ = params._wheel_radius_;
  this->gear_ratio_ = params._gear_ratio_;
}

void EKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  this->imu_data_ = common_lib::sensor_data::ImuData(0, 0, 0);
  if (this->imu_data_received_ && this->wss_data_received_ && this->motor_rpm_received_ &&
      this->steering_angle_received_) {
    std::cout << "1 - State: " << this->state_(0) << " " << this->state_(1) << " "
              << this->state_(2) << std::endl;
    this->predict(this->state_, this->covariance_, this->process_noise_matrix_, this->last_update_,
                  this->imu_data_);
    std::cout << "2 - State: " << this->state_(0) << " " << this->state_(1) << " "
              << this->state_(2) << std::endl;
    this->correct(this->state_, this->covariance_, this->wss_data_, this->motor_rpm_,
                  this->steering_angle_);
    std::cout << "3 - State: " << this->state_(0) << " " << this->state_(1) << " "
              << this->state_(2) << std::endl;
  }
  this->imu_data_received_ = true;
  this->last_update_ = std::chrono::high_resolution_clock::now();
}

void EKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  this->wss_data_received_ = true;
}

void EKF::motor_rpm_callback(double motor_rpm) {
  this->motor_rpm_ = motor_rpm;
  this->motor_rpm_received_ = true;
}

void EKF::steering_callback(double steering_angle) {
  this->steering_angle_ = steering_angle;
  this->steering_angle_received_ = true;
}

common_lib::structures::Velocities EKF::get_velocities() {
  common_lib::structures::Velocities velocities;
  velocities.velocity_x = this->state_(0);
  velocities.velocity_y = this->state_(1);
  velocities.rotational_velocity = this->state_(2);
  return velocities;
}

void EKF::predict(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
                  const Eigen::Matrix3f& process_noise_matrix,
                  const std::chrono::high_resolution_clock::time_point last_update,
                  common_lib::sensor_data::ImuData& imu_data) {
  auto current_time_point = std::chrono::high_resolution_clock::now();
  auto dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time_point - last_update)
          .count();
  auto jacobian = motion_lib::particle_model::jacobian_of_velocity_update();
  covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
  motion_lib::particle_model::update_velocities(
      state, imu_data.acceleration_x, imu_data.acceleration_y, imu_data.rotational_velocity, dt);
}

void EKF::correct(Eigen::Vector3f& state, Eigen::Matrix3f& covariance,
                  common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
                  double steering_angle) {
  Eigen::VectorXf predicted_observations = observation_lib::bicycle_model::estimate_observations(
      state, this->wheel_base_, this->weight_distribution_front_, this->gear_ratio_,
      this->wheel_radius_);
  Eigen::VectorXf observations = Eigen::VectorXf::Zero(6);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, wss_data.rl_rpm, wss_data.rr_rpm,
      steering_angle, motor_rpm;
  std::cout << "Predicted observations: ";
  for (int i = 0; i < predicted_observations.rows(); i++) {
    std::cout << predicted_observations(i) << " ";
  }
  std::cout << std::endl;
  std::cout << "Observations: ";
  for (int i = 0; i < observations.rows(); i++) {
    std::cout << observations(i) << " ";
  }
  std::cout << std::endl;
  Eigen::VectorXf y = observations - predicted_observations;
  std::cout << "y: ";
  for (int i = 0; i < y.rows(); i++) {
    std::cout << y(i) << " ";
  }
  std::cout << std::endl;
  Eigen::MatrixXf jacobian = observation_lib::bicycle_model::jacobian_of_observation_estimation(
      state, this->wheel_base_, this->weight_distribution_front_, this->gear_ratio_,
      this->wheel_radius_);
  std::cout << "Jacobian: " << std::endl;
  for (int i = 0; i < jacobian.rows(); i++) {
    for (int j = 0; j < jacobian.cols(); j++) {
      std::cout << jacobian(i, j) << " ";
    }
    std::cout << std::endl;
  }

  Eigen::MatrixXf kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->measurement_noise_matrix_).inverse();
  std::cout << "Kalman gain: ";
  for (int i = 0; i < kalman_gain.rows(); i++) {
    std::cout << kalman_gain(i, 0) << " ";
  }
  std::cout << std::endl;
  Eigen::Vector3f dx = kalman_gain * y;
  std::cout << "dx: ";
  for (int i = 0; i < dx.rows(); i++) {
    std::cout << dx(i) << " ";
  }
  std::cout << std::endl;
  state += kalman_gain * y;
  covariance = (Eigen::Matrix3f::Identity() - kalman_gain * jacobian) * covariance;
}