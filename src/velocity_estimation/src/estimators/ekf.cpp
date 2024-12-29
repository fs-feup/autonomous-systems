#include "estimators/ekf.hpp"

EKF::EKF(const VEParameters& params) {
  this->process_noise_matrix_ = Eigen::Matrix3f::Identity() * params._ekf_process_noise_;
}

void EKF::IMUCallback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  if (this->imu_data_received_ && this->wss_data_received_ && this->resolver_data_received_ &&
      this->steering_data_received_) {
    this->predict(this->state_, this->covariance_, this->process_noise_matrix_, this->last_update_,
                  this->imu_data_);
    // TODO: correct(this->state_, this->covariance_, this->wss_data_, this->resolver_data_,
  }
  this->imu_data_received_ = true;
  this->last_update_ = std::chrono::high_resolution_clock::now();
}

void EKF::WSSCallback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  this->wss_data_received_ = true;
}

void EKF::ResolverCallback(double resolver_data) {
  this->resolver_data_ = resolver_data;
  this->resolver_data_received_ = true;
}

void EKF::SteeringCallback(double steering_data) {
  this->steering_data_ = steering_data;
  this->steering_data_received_ = true;
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