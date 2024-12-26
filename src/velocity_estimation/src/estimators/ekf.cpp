#include "estimators/ekf.hpp"

void EKF::IMUCallback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
}

void EKF::WSSCallback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
}

void EKF::ResolverCallback(double resolver_data) { this->resolver_data_ = resolver_data; }

void EKF::SteeringCallback(double steering_data) { this->steering_data_ = steering_data; }

common_lib::structures::Velocities EKF::get_velocities() {
  common_lib::structures::Velocities velocities;
  velocities.velocity_x = this->state_(0);
  velocities.velocity_y = this->state_(1);
  velocities.rotational_velocity = this->state_(2);
  return velocities;
}