#include "estimators/no_rear_wss_ekf.hpp"

#include <cmath>

NoRearWSSEKF::NoRearWSSEKF(const VEParameters& params) {
  this->_process_noise_matrix_ = Eigen::Matrix3d::Zero();
  this->_process_noise_matrix_(0, 0) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(1, 1) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(2, 2) = params.angular_velocity_process_noise_;
  this->_angular_velocity_process_noise_multiplier_ =
      params.angular_velocity_process_noise_multiplier_;
  this->_wheels_measurement_noise_matrix_ = Eigen::MatrixXd::Identity(4, 4);
  this->_wheels_measurement_noise_matrix_(0, 0) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(1, 1) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(2, 2) = params.steering_angle_noise_;
  this->_wheels_measurement_noise_matrix_(3, 3) = params.motor_rpm_noise_;
  this->_imu_measurement_noise_matrix_ = Eigen::MatrixXd(1, 1);
  this->_imu_measurement_noise_matrix_(0, 0) = params.imu_rotational_noise_;
  this->car_parameters_ = params.car_parameters_;
  this->s2v_model = s2v_models_map.at(params._s2v_model_name_)(params.car_parameters_);
  this->process_model = vel_process_models_map.at(params._process_model_name_)();
}

void NoRearWSSEKF::imu_accel_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "Before Prediction: %f %f %f",
               this->_state_(0), this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "Before Prediction: \n"
                                                                     << this->_covariance_);
  if (!this->imu_data_received_) {
    this->imu_data_received_ = true;
  } else {
    this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
                  this->_last_update_, this->imu_data_);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "After Prediction - State: %f %f %f",
               this->_state_(0), this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "After Prediction - Covariance: \n"
                                                                     << this->_covariance_);
  this->_last_update_ = imu_data.timestamp_;
  this->_last_predict_ = imu_data.timestamp_;
}

void NoRearWSSEKF::imu_angular_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;

  this->correct_imu(this->_state_, this->_covariance_, this->imu_data_);
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "After IMU Correction - State: %f %f %f",
               this->_state_(0), this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "After IMU Correction - Covariance: \n"
                          << this->_covariance_);
  this->_last_update_ = imu_data.timestamp_;
}

void NoRearWSSEKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  this->wss_data_received_ = true;
  if (this->steering_angle_received_ && this->motor_rpm_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
                 "After WSS Correction - State: %f %f %f", this->_state_(0), this->_state_(1),
                 this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                        "After WSS Correction - Covariance: \n"
                            << this->_covariance_);
  }
  this->_last_update_ = wss_data.timestamp_;
}

void NoRearWSSEKF::motor_rpm_callback(double motor_rpm) {
  this->motor_rpm_ = motor_rpm;
  this->motor_rpm_received_ = true;
  // if (this->steering_angle_received_ && this->wss_data_received_) {
  //   this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
  //                        this->steering_angle_);
  //   this->wss_data_received_ = false;
  //   this->steering_angle_received_ = false;
  //   this->motor_rpm_received_ = false;
  //   RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
  //                "After Motor RPM Correction - State: %f %f %f", this->_state_(0),
  //                this->_state_(1), this->_state_(2));
  //   RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
  //                       "After Motor RPM Correction - Covariance: \n"
  //                           << this->_covariance_);
  // }
}

void NoRearWSSEKF::steering_callback(double steering_angle) {
  this->steering_angle_ = steering_angle;
  this->steering_angle_received_ = true;
  // if (this->wss_data_received_ && this->motor_rpm_received_) {
  //   this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
  //                        this->steering_angle_);
  //   this->wss_data_received_ = false;
  //   this->steering_angle_received_ = false;
  //   this->motor_rpm_received_ = false;
  //   RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
  //                "After Steering Angle Correction - State: %f %f %f", this->_state_(0),
  //                this->_state_(1), this->_state_(2));
  //   RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
  //                       "After Steering Angle Correction - Covariance: \n"
  //                           << this->_covariance_);
  // }
}

common_lib::structures::Velocities NoRearWSSEKF::get_velocities() {
  common_lib::structures::Velocities velocities;
  velocities.velocity_x = this->_state_(0);
  velocities.velocity_y = this->_state_(1);
  velocities.rotational_velocity = this->_state_(2);
  velocities.timestamp_ = this->_last_update_;
  velocities.velocity_x_noise_ = this->_covariance_(0, 0);
  velocities.velocity_y_noise_ = this->_covariance_(1, 1);
  velocities.rotational_velocity_noise_ = this->_covariance_(2, 2);
  return velocities;
}

void NoRearWSSEKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                           const Eigen::Matrix3d& process_noise_matrix,
                           const rclcpp::Time last_predict,
                           common_lib::sensor_data::ImuData& imu_data) {
  double dt = (imu_data.timestamp_ - last_predict).seconds();
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "IMU Timestamp: %f, Last Predict: %f, dt: %f", imu_data.timestamp_.seconds(),
               last_predict.seconds(), dt);
  Eigen::Vector3d accelerations(imu_data.acceleration_x, imu_data.acceleration_y, 0.0);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict - dt: " << dt << ", Accel X: " << imu_data.acceleration_x
                                       << ", Accel Y: " << imu_data.acceleration_y
                                       << ", Gyro Z: " << imu_data.rotational_velocity);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict - Process noise matrix: \n"
                          << process_noise_matrix);

  Eigen::Matrix3d jacobian = this->process_model->get_jacobian_velocities(state, accelerations, dt);
  Eigen::Matrix3d process_noise_jacobian =
      this->process_model->get_jacobian_sensor_data(state, accelerations, dt);
  covariance = jacobian * covariance * jacobian.transpose() +
               process_noise_jacobian * process_noise_matrix * process_noise_jacobian.transpose();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict - Jacobian: \n"
                          << jacobian << "\nProcess Noise Jacobian: \n"
                          << process_noise_jacobian);
  state = this->process_model->get_next_velocities(state, accelerations, dt);
  this->_has_made_prediction_ = true;
}

void NoRearWSSEKF::correct_wheels(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                                  common_lib::sensor_data::WheelEncoderData& wss_data,
                                  double motor_rpm, double steering_angle) {
  Eigen::VectorXd predicted_observations = this->s2v_model->cg_velocity_to_wheels(state);
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(4);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, steering_angle, motor_rpm;
  Eigen::VectorXd y = observations - predicted_observations;
  Eigen::MatrixXd jacobian = this->s2v_model->jacobian_cg_velocity_to_wheels(state);
  Eigen::MatrixXd observations_diag = observations.asDiagonal();
  Eigen::MatrixXd scaled_noise_matrix = observations_diag * this->_wheels_measurement_noise_matrix_;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "correct_wheels - Scaled noise matrix: \n"
                          << scaled_noise_matrix);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "correct_wheels - observations diag: \n"
                          << observations_diag);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "correct_wheels - Original noise matrix: \n"
                          << this->_wheels_measurement_noise_matrix_);
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + scaled_noise_matrix).inverse();

  kalman_gain(2, 0) = 0;
  kalman_gain(2, 1) = 0;
  kalman_gain(2, 2) = 0;
  kalman_gain(2, 3) = 0;

  // DEBUG PRINTS
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Predicted observations: %f %f %f %f", predicted_observations(0),
               predicted_observations(1), predicted_observations(2), predicted_observations(3));
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Observations: %f %f %f %f", observations(0), observations(1),
               observations(2), observations(3));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Jacobian: \n"
                                                                     << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Kalman gain: \n"
                                                                     << kalman_gain);
  Eigen::Vector3d correction = kalman_gain * y;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Correction: \n"
                                                                     << correction);
  state += kalman_gain * y;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - New State: \n"
                                                                     << state);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Some Matrix: \n"
                                                                     << kalman_gain * jacobian);
  // if (this->_has_made_prediction_)
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}

void NoRearWSSEKF::correct_imu(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                               common_lib::sensor_data::ImuData& imu_data) {
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_imu - IMU rotational velocity: %f, State rotational velocity: %f",
               imu_data.rotational_velocity, state(2));
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
  Eigen::Vector3d correction = kalman_gain * y;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Correction: \n"
                                                                     << correction);
  state += kalman_gain * y;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - New State: \n"
                                                                     << state);
  covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}