#include "estimators/ekf.hpp"

EKF::EKF(const VEParameters& params) {
  this->_process_noise_matrix_ = Eigen::Matrix3d::Zero();
  this->_process_noise_matrix_(0, 0) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(1, 1) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(2, 2) = params.angular_velocity_process_noise_;
  this->_wheels_measurement_noise_matrix_ = Eigen::MatrixXd::Identity(6, 6);
  this->_wheels_measurement_noise_matrix_(0, 0) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(1, 1) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(2, 2) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(3, 3) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(4, 4) = params.steering_angle_noise_;
  this->_wheels_measurement_noise_matrix_(5, 5) = params.motor_rpm_noise_;
  this->_imu_measurement_noise_matrix_ = Eigen::MatrixXd(1, 1);
  this->_imu_measurement_noise_matrix_(0, 0) = params.imu_rotational_noise_;
  this->car_parameters_ = params.car_parameters_;
  this->s2v_model = s2v_models_map.at(params._s2v_model_name_)(params.car_parameters_);
  this->process_model = vel_process_models_map.at(params._process_model_name_)();
}

void EKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "1 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "1 - Covariance: \n"
                                                                     << this->_covariance_);
  if (!this->imu_data_received_) {
    this->imu_data_received_ = true;
  } else {
    this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
                  this->_last_update_, this->imu_data_);
  }
  this->_last_update_ = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "2 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "2 - Covariance: \n"
                                                                     << this->_covariance_);

  this->correct_imu(this->_state_, this->_covariance_, this->imu_data_);
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                     << this->_covariance_);
}

void EKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  this->wss_data_received_ = true;
  if (this->steering_angle_received_ && this->motor_rpm_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

void EKF::motor_rpm_callback(double motor_rpm) {
  this->motor_rpm_ = motor_rpm;
  this->motor_rpm_received_ = true;
  if (this->steering_angle_received_ && this->wss_data_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

void EKF::steering_callback(double steering_angle) {
  this->steering_angle_ = steering_angle;
  this->steering_angle_received_ = true;
  if (this->wss_data_received_ && this->motor_rpm_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

common_lib::structures::Velocities EKF::get_velocities() {
  common_lib::structures::Velocities velocities;
  velocities.velocity_x = this->_state_(0);
  velocities.velocity_y = this->_state_(1);
  velocities.rotational_velocity = this->_state_(2);
  velocities.timestamp_ = rclcpp::Clock().now();
  velocities.velocity_x_noise_ = this->_covariance_(0, 0);
  velocities.velocity_y_noise_ = this->_covariance_(1, 1);
  velocities.rotational_velocity_noise_ = this->_covariance_(2, 2);
  return velocities;
}

void EKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                  const Eigen::Matrix3d& process_noise_matrix, const rclcpp::Time last_update,
                  common_lib::sensor_data::ImuData& imu_data) {
  rclcpp::Time current_time_point =
      rclcpp::Clock().now();  // TODO: change calculation to use message timestamps
  double dt = (current_time_point - last_update).seconds();
  Eigen::Vector3d accelerations(imu_data.acceleration_x, imu_data.acceleration_y, 0.0);

  // Process noise for angular velocity greater, the greater the angular velocity
  Eigen::Matrix3d actual_process_noise_matrix = process_noise_matrix;
  // actual_process_noise_matrix(2, 2) += process_noise_matrix(0, 0) * state(2) * dt * 1000.0;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict - Process noise matrix: \n"
                          << actual_process_noise_matrix);

  Eigen::Matrix3d jacobian = this->process_model->get_jacobian_velocities(state, accelerations, dt);
  covariance = jacobian * covariance * jacobian.transpose() + actual_process_noise_matrix;
  state = this->process_model->get_next_velocities(state, accelerations, dt);
}

void EKF::correct_wheels(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
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
      (jacobian * covariance * jacobian.transpose() + this->_wheels_measurement_noise_matrix_)
          .inverse();

  // DEBUG PRINTS
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Predicted observations: %f %f %f %f %f %f",
               predicted_observations(0), predicted_observations(1), predicted_observations(2),
               predicted_observations(3), predicted_observations(4), predicted_observations(5));
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Observations: %f %f %f %f %f %f", observations(0), observations(1),
               observations(2), observations(3), observations(4), observations(5));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Jacobian: \n"
                                                                     << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Kalman gain: \n"
                                                                     << kalman_gain);
  state += kalman_gain * y;
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}

void EKF::correct_imu(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                      common_lib::sensor_data::ImuData& imu_data) {
  Eigen::VectorXd y = Eigen::VectorXd(1);
  y(0) = imu_data.rotational_velocity - state(2);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd(1, 3);
  jacobian(0, 0) = 0;
  jacobian(0, 1) = 0;
  jacobian(0, 2) = 1;
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->_imu_measurement_noise_matrix_)
          .inverse();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Jacobian: \n"
                                                                     << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Kalman gain: \n"
                                                                     << kalman_gain);
  state += kalman_gain * y;
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}