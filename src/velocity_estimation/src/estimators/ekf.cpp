#include "estimators/ekf.hpp"

#include <iostream>

EKF::EKF(const VEParameters& params) {
  this->process_noise_matrix_ = Eigen::Matrix3d::Identity() * params._ekf_process_noise_;
  this->measurement_noise_matrix_ =
      Eigen::MatrixXd::Identity(6, 6) * params._ekf_measurement_noise_;
  this->wheel_base_ = params._wheel_base_;
  this->weight_distribution_front_ = params._weight_distribution_front_;
  this->wheel_radius_ = params._wheel_radius_;
  this->gear_ratio_ = params._gear_ratio_;
}

void EKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  if (this->imu_data_received_ && this->wss_data_received_ && this->motor_rpm_received_ &&
      this->steering_angle_received_) {
    this->predict(this->state_, this->covariance_, this->process_noise_matrix_, this->last_update_,
                  this->imu_data_);
    this->correct(this->state_, this->covariance_, this->wss_data_, this->motor_rpm_,
                  this->steering_angle_);
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

void EKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                  const Eigen::Matrix3d& process_noise_matrix,
                  const std::chrono::high_resolution_clock::time_point last_update,
                  common_lib::sensor_data::ImuData& imu_data) {
  auto current_time_point = std::chrono::high_resolution_clock::now();
  auto dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time_point - last_update)
          .count();
  CAParticleModel cvparticle_model = CAParticleModel();
  auto jacobian = cvparticle_model.jacobian_of_velocity_update();
  covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
  cvparticle_model.update_velocities(state, imu_data.acceleration_x, imu_data.acceleration_y,
                                     imu_data.rotational_velocity, dt);
}

void EKF::correct(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                  common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
                  double steering_angle) {
  BicycleModel bicycle_model = BicycleModel(common_lib::car_parameters::CarParameters());
  Eigen::VectorXd predicted_observations = bicycle_model.cg_velocity_to_wheels(state);
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(6);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, wss_data.rl_rpm, wss_data.rr_rpm,
      steering_angle, motor_rpm;
  Eigen::VectorXd y = observations - predicted_observations;
  Eigen::MatrixXd jacobian = bicycle_model.jacobian_cg_velocity_to_wheels(state);
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->measurement_noise_matrix_).inverse();
  state += kalman_gain * y;
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}