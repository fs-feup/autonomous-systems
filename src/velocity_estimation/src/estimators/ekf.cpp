#include "estimators/ekf.hpp"

EKF::EKF(const VEParameters& params) {
  this->process_noise_matrix_ = Eigen::Matrix3d::Identity();
  this->process_noise_matrix_(0, 0) = params.imu_acceleration_noise_;
  this->process_noise_matrix_(1, 1) = params.imu_acceleration_noise_;
  this->process_noise_matrix_(2, 2) = params.imu_rotational_noise_;
  this->measurement_noise_matrix_ = Eigen::MatrixXd::Identity(6, 6);
  this->measurement_noise_matrix_(0, 0) = params.wheel_speed_noise_;
  this->measurement_noise_matrix_(1, 1) = params.wheel_speed_noise_;
  this->measurement_noise_matrix_(2, 2) = params.wheel_speed_noise_;
  this->measurement_noise_matrix_(3, 3) = params.wheel_speed_noise_;
  this->measurement_noise_matrix_(4, 4) = params.steering_angle_noise_;
  this->measurement_noise_matrix_(5, 5) = params.motor_rpm_noise_;
  this->car_parameters_ = params.car_parameters_;
  this->s2v_model = s2v_models_map.at(params._s2v_model_name_)(params.car_parameters_);
}

void EKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  if (this->imu_data_received_ && this->wss_data_received_ && this->motor_rpm_received_ &&
      this->steering_angle_received_) {
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "1 - State: %f %f %f", this->state_(0),
                 this->state_(1), this->state_(2));
    this->predict(this->state_, this->covariance_, this->process_noise_matrix_, this->_last_update_,
                  this->imu_data_);
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "2 - State: %f %f %f", this->state_(0),
                 this->state_(1), this->state_(2));
    //this->correct(this->state_, this->covariance_, this->wss_data_, this->motor_rpm_,
                  //sthis->steering_angle_);
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->state_(0),
                 this->state_(1), this->state_(2));
  }
  this->imu_data_received_ = true;
  this->_last_update_ = rclcpp::Clock().now();
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
  velocities.timestamp_ = rclcpp::Clock().now();
  velocities.velocity_x_noise_ = this->covariance_(0, 0);
  velocities.velocity_y_noise_ = this->covariance_(1, 1);
  velocities.rotational_velocity_noise_ = this->covariance_(2, 2);
  return velocities;
}

void EKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                  const Eigen::Matrix3d& process_noise_matrix, const rclcpp::Time last_update,
                  common_lib::sensor_data::ImuData& imu_data) {
  rclcpp::Time current_time_point = rclcpp::Clock().now();
  double dt = (current_time_point - last_update).seconds();
  CAParticleModel cvparticle_model = CAParticleModel();
  Eigen::Matrix3d jacobian = cvparticle_model.jacobian_of_velocity_update();
  covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
  cvparticle_model.update_velocities(state, imu_data.acceleration_x, imu_data.acceleration_y,
                                     imu_data.rotational_velocity, dt);
}

void EKF::correct(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                  common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
                  double steering_angle) {
  Eigen::VectorXd predicted_observations = this->s2v_model->cg_velocity_to_wheels(state);
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(6);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, wss_data.rl_rpm, wss_data.rr_rpm,
      steering_angle, motor_rpm;
  Eigen::VectorXd y = observations - predicted_observations;
  Eigen::MatrixXd jacobian = this->s2v_model->jacobian_cg_velocity_to_wheels(state);
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->measurement_noise_matrix_).inverse();

  // DEBUG PRINTS
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "Predicted observations: %f %f %f %f %f %f", predicted_observations(0),
               predicted_observations(1), predicted_observations(2), predicted_observations(3),
               predicted_observations(4), predicted_observations(5));
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "Observations: %f %f %f %f %f %f",
               observations(0), observations(1), observations(2), observations(3), observations(4),
               observations(5));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "Covariance: \n" << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "Jacobian: \n" << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "Kalman gain: \n" << kalman_gain);
  state += kalman_gain * y;
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}